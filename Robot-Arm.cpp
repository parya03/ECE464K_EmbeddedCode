#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <iostream>         
// #include <chrono>
#include "Eigen/Dense"
#include <math.h>
#include <sys/cdefs.h>
#include "projdefs.h"
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"
#include "Servo.hpp"
#include "FreeRTOS.h"
#include "robot.pb.h"
#include "stream_buffer.h"
#include "Communication.hpp"
#include <queue>
#include <array>


#define SQUARE(x) ((x) * (x))

using namespace std;
using namespace Eigen;

using JointArray = std::array<float, 5>;

// Motor angles
// base arm1 arm2 pitch gripper_angle
JointArray current_angles{};
std::queue<JointArray> motor_angles_queue;

/* TODOs
 * Make error take new position into account
 * Fix communication buffer overflow
 * Latency optimization
 * */

// Define manipulator.
// This is the DH parameters for the KUKA KR6 robot
auto R = std::make_shared<quik::Robot<3>>(
	// Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
	(Matrix<double, 3, 4>() <<
		0,    M_PI/2,        10.22,       0,
		15.24,   0,         0,            M_PI/2,
		17.06,   0,    0,           0).finished(), // Includes arm2 and claw (arm2 is 10.56cm, claw is 6.5)
    // (Matrix<double, 3, 4>() <<
	// 	0,    M_PI/2,        6.3,       0,
	// 	15.24f,   0,         0,            M_PI/2,
	// 	10.33f,   0,    0,           0).finished(),
					  
	// Second argument is a list of joint types
	// true is prismatic, false is revolute
	// KUKA KR6 only has revolute joints
	(Vector<quik::JOINTTYPE_t,3>() << 
        quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE
    ).finished(),

	// Third agument is a list of joint directions
	// Allows you to change the sign (direction) of
	// the joints.
	(Vector<double,3>(3) << 1, 1, 1).finished(),

	// Fourth and fifth arguments are the base and tool transforms, respectively
	Matrix4d::Identity(4,4),
	Matrix4d::Identity(4,4)
);

// Define the IK options
const quik::IKSolver<3> IKS(
    R, // The robot object (pointer)
    50, // max number of iterations
    quik::ALGORITHM_QUIK, // quik::ALGORITHM_QUIK, // algorithm (ALGORITHM_QUIK, ALGORITHM_NR or ALGORITHM_BFGS)
    1e-12, // Exit tolerance
    1e-14, // Minimum step tolerance
    0.005, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
    10, // max consequitive gradient fails
    80, // Max gradient fails
    0.1, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
    0.34, // 0.34, // Max linear error step
    1 // Max angular error step
);

bool run_state = true;

double min_sqrt_normed_err = (double)__FLT_MAX__; // Big double init because err is (hopefully) less
Vector3d min_err_joint_angles;

// Data for hand position that we are currently working with
// Updated and recieved from stream buffer
handdata_t curr_position = {
    .timestamp = 0.0f,
    .x = 18.0f,
    .y = 0.0f,
    .z = 25.0f, // Start basically straight up (zero joint angle)
    .openness = 0.0f,
    .pitch = 0.0f,
};

Servo base(2, 0);
Servo arm1(3, 0, true);
Servo arm2(4, 90);
Servo wrist(5, 0);
Servo gripper(6, 0);  

void RobotArm_Task(void *pvParameters) {
    printf("Robot Arm task started\n");

    base.startPWMControllers();
    arm1.startPWMControllers();
    arm2.startPWMControllers();
    wrist.startPWMControllers();
    gripper.startPWMControllers();

    // Set up a UART RX interrupt
    // And set up and enable the interrupt handlers
    // irq_set_exclusive_handler(UART0_IRQ, uart_rx_interrupt);
    // irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    // uart_set_irq_enables(uart0, true, false);
    // while(1) {
    //     base.zero();
    //     arm1.zero();
    //     arm2.zero();
    // }
    
    // Wait on communication stream buffer
    while(!communication_message_buf) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // while(1) {
    //     base.zero();
    //     arm1.zero();
    //     arm2.zero();
    //     wrist.zero();
    //     gripper.zero();
    // }

    Vector3d Q_prev(3.14, -3.26, 3.76);
    // Matrix3d Q_prev {{3.6}, {-1.6}, {3.14}};
    // Q_prev.setRandom(R->dof, 1);
    // Q_prev.setRandom();
    // Q_prev << 3.6, -1.6, 3.14; // Give a bad initial guess for testing

    printf("PWM started\n");

    while(1) {
        // Initilize variables
        int N = 1; // Number of poses to generate
        int DOF = R->dof;
        Matrix<double,3,Dynamic>    Q(DOF, N),	    // True joint angles
                                    Q0(DOF, N),	    // Initial guess of joint angles
                                    Q_star(DOF, N);	// Solver's solution
        Matrix<double,6,Dynamic>    e_star(6,N);	// Error at solver pose
        std::vector<int>            iter(N);	    // Store number of iterations of algorithm
        std::vector<quik::BREAKREASON_t>  breakReason(N);	// Store break out reason
        Matrix4d                    T,		        // True forward kinematics transform
                                    T_star;	        // Forward kinematics at solver solution
        
        Matrix4d    Tn(4,4); 	    // 4N * 4 matrix of vertically stacked transforms to be solved.
                                                    // This is just a convenient way of sending in an array of transforms.

        // Update current data based off of stream buffer if data is available
        // Don't block - if data isn't available keep going with what we currently have
        auto mb_bytes_received = xMessageBufferReceive(communication_message_buf, &curr_position, sizeof(handdata_t), 0);
        // if(mb_bytes_received) {
        //     printf("Receieved %d bytes from SB\n", mb_bytes_received);
        //     printf("Setting target position to to %f, %f, %f, wrist to %f and openness to %f\n", curr_position.x, curr_position.y, curr_position.z, curr_position.pitch, curr_position.openness);
        //     min_sqrt_normed_err = (double)__FLT_MAX__; // Reset the error if new coordinate received
        // }
        // Perturb true answers slightly to get initial "guess" (knock over by 0.1 radians)
        // Q0 = Q_prev.array();
        Q0 = Q_prev.array() + 0.01;
        // Q0 = Q_prev;
        // Q0.setZero();

        printf("Q0 - %d x %d:\n", Q0.rows(), Q0.cols());
        for(int i = 0; i < DOF; i++) {
            printf("%f ", Q0(i));
        }
        printf("\n");

        // 25.456 because that would make a right triangle with high on potenuse = 36 (robot length) according to Pythagoras
        // double Tn_xyz[3] = {20.0f, 0.0f, 20.0f};
        // double Tn_xyz[3] = {4000.0f, 0.0f, 4000.0f};
        // double pitch = 0.0f; // -90 - 90
        Vector3d Tn_xyz {{curr_position.x, curr_position.y, curr_position.z}};
        double pitch = curr_position.pitch;

        // Rotation done by taking given pitch into account in Y axis (Y-axis rotation is X-axis pitch),
        // then matmul that Y-axis rotation with whichever other rotation we need (ex. Y axis)
        // https://opentextbooks.clemson.edu/wangrobotics/chapter/forward-kinematics/ section 2.2.4
        Vector3d Tn_norm = Tn_xyz.normalized(); // We would get angles using tan anyways, so sin and cos of tan is just lengths
        Matrix3d X_rot {{1, 0, 0}, {0, Tn_norm[1], -Tn_norm[2]}, {0, Tn_norm[2], Tn_norm[1]}};
        // Matrix3d X_rot {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        Matrix3d Y_rot {{Tn_norm[0], 0, Tn_norm[2]}, {0, 1, 0}, {-Tn_norm[2], 0, Tn_norm[0]}};
        // Matrix3d Y_rot {{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}};
        Matrix3d Z_rot {{Tn_norm[0], -Tn_norm[1], 0}, {Tn_norm[1], Tn_norm[0], 0}, {0, 0, 1}};
        Matrix3d rot_matrix = X_rot * Y_rot * Z_rot;
        // Matrix3d X_rot = Identity(4, 4);
        Tn << rot_matrix(0, 0), rot_matrix(0, 1), rot_matrix(0, 2), Tn_xyz[0], \
                rot_matrix(1, 0), rot_matrix(1, 1), rot_matrix(1, 2), Tn_xyz[1], \ 
                rot_matrix(2, 0), rot_matrix(2, 1), rot_matrix(2, 2), Tn_xyz[2], \
                0, 0, 0, 1;

        // Tn << 0, 0, 0, Tn_xyz[0], \
        //     0, 0, 0, Tn_xyz[1], \ 
        //     0, 0, 0, Tn_xyz[2], \
        //     0, 0, 0, 1;

        // Initial guess:
        // Straight vector from base to point but as joint angles
        // Add a small perturbation or the library will return NaN
        printf("Tn_norm: %f, %f, %f\n", Tn_norm[0], Tn_norm[1], Tn_norm[2]);
        // Q0(0) = atan2(Tn_norm[1], Tn_norm[0]) + 0.01; // ∠ base = atan(y length / x length)
        // Q0(1) = atan2(Tn_norm[2], sqrt(SQUARE(Tn_norm[0]) + SQUARE(Tn_norm[1]))) + 0.01; // ∠ arm1 = atan(z / len(x + y vectors))
        // Q0(2) = 0 + 0.01; // Guess that it has zero angle for now
        // R->print();

        printf("Q0: %f %f %f, sqrt is %f\n", Q0(0), Q0(1), Q0(2), sqrt(SQUARE(Tn_norm[0]) + SQUARE(Tn_norm[1])));
        
        // sleep_ms(1000);
        
        // printf("Hit\n");
        // Start a timer, to get time
        // auto startTime = chrono::high_resolution_clock::now();
        auto startTime = pdTICKS_TO_MS(xTaskGetTickCount());

        // Solve using the IK function, store results in Q_star, e_star, iter and breakreason
        IKS.IK(Tn, Q0, Q_star, e_star, iter, breakReason);

        // Get the time after calling the IK function
        // chrono::duration<double, std::micro> elapsed = chrono::high_resolution_clock::now() - startTime;
        auto endTime = pdTICKS_TO_MS(xTaskGetTickCount());

        // Print out results
        // printf("Joint angles (start):\n");
        // for (int i = 0; i < Q0.rows(); i++) {
        //     for (int j = 0; j < Q0.cols(); j++) {
        //         printf("%f ", Q0(i, j));
        //     }
        //     printf("\n");
        // }
        // printf("\n");

        // printf("Joint angles (true):\n");
        // for (int i = 0; i < Q.rows(); i++) {
        //     for (int j = 0; j < Q.cols(); j++) {
        //         printf("%f ", Q(i, j));
        //     }
        //     printf("\n");
        // }
        // printf("\n");

        // printf("The final joint angles are:\n");
        // for (int i = 0; i < Q_star.rows(); i++) {
        //     for (int j = 0; j < Q_star.cols(); j++) {
        //         printf("Q_star[%d, %d]: %f = %f Pi", i, j, Q_star(i, j), Q_star(i, j)/double(M_PI));
        //     }
        //     printf("\n");
        // }
        // printf("\n");
        
        /**
         * enum BREAKREASON_t : uint8_t {
            BREAKREASON_TOLERANCE = 0, // Tolerance reached
            BREAKREASON_MIN_STEP, // minimum step size is reached
            BREAKREASON_MAX_ITER, // Max iterations reached
            BREAKREASON_GRAD_FAILS // Gradient failed to improve
            };
         **/

        printf("Break reason is:\n");
        for (const auto& reason : breakReason) {
            printf("%d ", reason);
        }
        printf("\n");

        // printf("Number of iterations:\n");
        // for (const auto& iter_i : iter) {
        //     printf("%d ", iter_i);
        // }
        // printf("\n");
        
        // printf("IK finished!\n");

        
        printf("Commanded transform (Tn):\n");
        for (int i = 0; i < Tn.rows(); i++) {
            for (int j = 0; j < Tn.cols(); j++) {
                printf("%f ", Tn(i, j));
            }
            printf("\n");
        }

        // Matrix4d T_fk;
        // R->FKn(Q_star.col(0), T_fk);

        VectorXd err_vec = e_star.col(0); // Only one column because only one pose
        // TODO: Define error as purely based on translation position rather than rotation
        // double normed_error = 0.0;
        // normed_error += SQUARE(Tn_xyz[0] - T_fk(0, 3));
        // normed_error += SQUARE(Tn_xyz[1] - T_fk(1, 3));
        // normed_error += SQUARE(Tn_xyz[2] - T_fk(2, 3));

        // for (auto i : err_vec) {
        //     normed_error += i * i; // Squared error
        // }
        // normed_error = (Tn(0, 3) * Q_star(0, 0))
        // printf("Final normed error for this run is: %8f, min recorded is %8f", sqrtf(normed_error), min_sqrt_normed_err);

        printf("Final normed error for this run is: %8f\n", sqrtf(e_star.col(0).norm()));
        // printf("\n");

        

        // Vector3d zero_vec = Vector3d::Zero();
        // R->FKn(zero_vec, T_fk);
        // printf("FK transformation matrix of computed angles (T_fk):\n");
        // for (int i = 0; i < T_fk.rows(); i++) {
        //     for (int j = 0; j < T_fk.cols(); j++) {
        //         printf("%f ", T_fk(i, j));
        //     }
        //     printf("\n");
        // }

        printf("Best joint angles so far:\n");
        for(auto i : Q_star.col(0)) {
            printf("%f \n", i);
        }
        printf("\n");
        //
        // R->FKn(min_err_joint_angles, T_fk);
        // printf("FK transformation matrix of best angles (T_fk):\n");
        // for (int i = 0; i < T_fk.rows(); i++) {
        //     for (int j = 0; j < T_fk.cols(); j++) {
        //         printf("%f ", T_fk(i, j));
        //     }
        //     printf("\n");
        // }

        printf("Total time taken: %d ms\n", endTime - startTime);

        // if(sqrtf(normed_error) < min_sqrt_normed_err) {
        //     min_sqrt_normed_err = sqrtf(normed_error);
        //     min_err_joint_angles = Q_star.col(0);
        //
        //
        // }
        min_err_joint_angles = Q_star.col(0);

        current_angles[0] = min_err_joint_angles(0, 0); // base angle
        current_angles[1] = min_err_joint_angles(1, 0); // arm 1 angle
        current_angles[2] = min_err_joint_angles(2, 0); // arm 2 angle
        current_angles[3] = pitch;
        float gripper_angle= 90.0*(1 - (curr_position.openness/100.0));
        current_angles[4] = gripper_angle;
        //motor_angles_queue.push(current_angles);

        int error = 0; // Are we able to reach the specified angles?
        // printf("Applying this transform because the error is least so far\n");
        // error |= base.setAngleRad(min_err_joint_angles(0, 0));
        // error |= arm1.setAngleRad(min_err_joint_angles(1, 0));
        // error |= arm2.setAngleRad(min_err_joint_angles(2, 0));
        // base.setAngleRad(0);
        // arm1.setAngleDegrees(0);
        // arm2.setAngleRad(0);
        
        error |= base.checkValidAngleDegrees(min_err_joint_angles(0, 0));
        error |= arm1.checkValidAngleDegrees(min_err_joint_angles(1, 0));
        error |= arm2.checkValidAngleDegrees(min_err_joint_angles(2, 0));

        // wrist.setAngleDegrees(pitch);
        //double gripper_angle = 90.0*(1 - (curr_position.openness/100.0));
        // gripper.setAngleDegrees(gripper_angle);

        // SHRUTI insert the angle write to the other thread somewhere here
        

        // wrist.setAngleDegrees(0);
        // gripper.setAngleDegrees(0);
        
        // base.print();
        // arm1.print();
        // arm2.print();

        Q_prev = Q_star;
        
        if(error) {
            printf("Could not reach specified joint angles - randomizing initial guess for next run\n");

            Q_prev.setRandom();
        }

        // sleep_ms(1000);
    }
	
    __unreachable();
	//return 0;
}

