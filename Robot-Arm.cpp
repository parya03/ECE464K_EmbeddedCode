#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <iostream>
// #include <chrono>
#include "Eigen/Dense"
#include <math.h>
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"
#include "Servo.hpp"

// int main()
// {
//     stdio_init_all();

//     while (true) {
//         printf("Hello, world!\n");
//         sleep_ms(1000);
//     }
// }

using namespace std;
using namespace Eigen;

// Define manipulator.
// This is the DH parameters for the KUKA KR6 robot
auto R = std::make_shared<quik::Robot<3>>(
	// Given as DOFx4 table, in the following order: a_i, alpha_i, d_i, theta_i.
	(Matrix<float, 3, 4>() <<
		0,    M_PI/2,        10.22f,       0,
		15.24f,   0,         0,            M_PI/2,
		10.56f,   0,    0,           0).finished(),
					  
	// Second argument is a list of joint types
	// true is prismatic, false is revolute
	// KUKA KR6 only has revolute joints
	(Vector<quik::JOINTTYPE_t,3>() << 
        quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE
    ).finished(),

	// Third agument is a list of joint directions
	// Allows you to change the sign (direction) of
	// the joints.
	(Vector<float,3>(3) << 1, 1, 1).finished(),

	// Fourth and fifth arguments are the base and tool transforms, respectively
	Matrix4f::Identity(4,4),
	Matrix4f::Identity(4,4)
);

// Define the IK options
const quik::IKSolver<3> IKS(
    R, // The robot object (pointer)
    2000, // max number of iterations
    quik::ALGORITHM_NR, // quik::ALGORITHM_QUIK, // algorithm (ALGORITHM_QUIK, ALGORITHM_NR or ALGORITHM_BFGS)
    1e-12, // Exit tolerance
    1e-14, // Minimum step tolerance
    0.005, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
    200, // max consequitive gradient fails
    800, // Max gradient fails
    1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
    0.34, // Max linear error step
    1 // Max angular error step
);

bool run_state = true;

// Used to read input data
void uart_rx_interrupt() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);

        if(ch == '0') {
            run_state = false;
        }
        if(ch == '1') {
            run_state = true;
        }
        // Can we send it back?
        if (uart_is_writable(uart0)) {
            // Change it slightly first!
            ch++;
            uart_putc(uart0, ch);
        }
        // chars_rxed++;
    }
}

float min_sqrt_normed_err = (float)__FLT_MAX__; // Big float init because err is (hopefully) less
Vector3f min_err_joint_angles;

int main() {
    stdio_init_all();

    // gpio_init(0);
    // gpio_set_pulls(0, false, true);
    // gpio_set_dir(0, GPIO_IN);

    // gpio_init(3);
    // gpio_set_pulls(3, false, true);
    // gpio_set_dir(3, GPIO_IN);

    // gpio_init(4);
    // gpio_set_pulls(4, false, true);
    // gpio_set_dir(4, GPIO_IN);

    // // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    // uint slice_num = pwm_gpio_to_slice_num(0);
    // uint32_t channel = pwm_gpio_to_channel(0);

    // // 1MHz PWM clock from divider
    // pwm_set_clkdiv_int_frac(slice_num, SYS_CLK_HZ / 1000000, 0);

    // // Set period of 50 ms
    // pwm_set_wrap(slice_num, 50000);
    // // Center servo
    // pwm_set_chan_level(slice_num, channel, 1500);

    // // Set initial B output high for three cycles before dropping
    // // pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
    
    // // Set the PWM running
    // pwm_set_enabled(slice_num, true);
    /// \end::setup_pwm[]

    Servo base(2, 0);
    Servo arm1(3, 0);
    Servo arm2(4, 0, true);

    base.startPWMControllers();
    arm1.startPWMControllers();
    arm2.startPWMControllers();

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

    Matrix<float,3,Dynamic> Q_prev;
    Q_prev.setRandom(R->dof, 1);

    while(1) {
        // Initilize variables
        int N = 1; // Number of poses to generate
        int DOF = R->dof;
        Matrix<float,3,Dynamic>    Q(DOF, N),	    // True joint angles
                                    Q0(DOF, N),	    // Initial guess of joint angles
                                    Q_star(DOF, N);	// Solver's solution
        Matrix<float,6,Dynamic>    e_star(6,N);	// Error at solver pose
        std::vector<int>            iter(N);	    // Store number of iterations of algorithm
        std::vector<quik::BREAKREASON_t>  breakReason(N);	// Store break out reason
        Matrix4f                    T,		        // True forward kinematics transform
                                    T_star;	        // Forward kinematics at solver solution
        
        Matrix4f    Tn(4,4); 	    // 4N * 4 matrix of vertically stacked transforms to be solved.
                                                    // This is just a convenient way of sending in an array of transforms.
        
        // Generate some random joint configurations for the robot
        // Q.setRandom(DOF, N);

        // Perturb true answers slightly to get initial "guess" (knock over by 0.1 radians)
        // Q0 = Q_prev.array();
        Q0 = Q_prev.array() + 0.01;
        // Q0 = Q_prev;

        // Do forward kinematics of each Q sample and store in the "tall" matrix
        // of transforms, Tn
        // for (int i = 0; i < N; i++){
        //     R->FKn( Q.col(i), T );
        //     Tn.middleRows<4>(i*4) = T;
        // }

        // 25.456 because that would make a right triangle with high on potenuse = 36 (robot length) according to Pythagoras
        Tn << 0, 0, 0, 36, \
                0, 0, 0, 0, \ 
                0, 0, 0, 0, \
                0, 0, 0, 1;

        // R->print();

        
        // sleep_ms(1000);
        
        // printf("Hit\n");
        // Start a timer, to get time
        // auto startTime = chrono::high_resolution_clock::now();
        auto startTime = to_ms_since_boot(get_absolute_time());

        // Solve using the IK function, store results in Q_star, e_star, iter and breakreason
        IKS.IK(Tn, Q0, Q_star, e_star, iter, breakReason);

        // Get the time after calling the IK function
        // chrono::duration<float, std::micro> elapsed = chrono::high_resolution_clock::now() - startTime;
        auto endTime = to_ms_since_boot(get_absolute_time());

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
        //         printf("Q_star[%d, %d]: %f = %f Pi", i, j, Q_star(i, j), Q_star(i, j)/float(M_PI));
        //     }
        //     printf("\n");
        // }
        // printf("\n");

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
        
        printf("IK finished!\n");

        VectorXf err_vec = e_star.col(0); // Only one column because only one pose
        float normed_error = 0.0;
        for (auto i : err_vec) {
            normed_error += i * i; // Squared error
        }
        printf("Final normed error for this run is: %f, min recorded is %f", sqrtf(normed_error), min_sqrt_normed_err);

        printf("\n");

        printf("Commanded transform (Tn):\n");
        for (int i = 0; i < Tn.rows(); i++) {
            for (int j = 0; j < Tn.cols(); j++) {
                printf("%f ", Tn(i, j));
            }
            printf("\n");
        }

        Matrix4f T_fk;
        R->FKn(Q_star.col(0), T_fk);
        // Vector3f zero_vec = Vector3f::Zero();
        // R->FKn(zero_vec, T_fk);
        printf("FK transformation matrix of computed angles (T_fk):\n");
        for (int i = 0; i < T_fk.rows(); i++) {
            for (int j = 0; j < T_fk.cols(); j++) {
                printf("%f ", T_fk(i, j));
            }
            printf("\n");
        }

        printf("Best joint angles so far:\n");
        for(auto i : min_err_joint_angles) {
            printf("%f \n", i);
        }
        printf("\n");

        R->FKn(min_err_joint_angles, T_fk);
        printf("FK transformation matrix of best angles (T_fk):\n");
        for (int i = 0; i < T_fk.rows(); i++) {
            for (int j = 0; j < T_fk.cols(); j++) {
                printf("%f ", T_fk(i, j));
            }
            printf("\n");
        }

        printf("Total time taken: %d ms\n", endTime - startTime);

        if(sqrtf(normed_error) < min_sqrt_normed_err) {
            min_sqrt_normed_err = sqrtf(normed_error);
            min_err_joint_angles = Q_star.col(0);

            
        }

        // printf("Applying this transform because the error is least so far\n");
        base.setAngleRad(min_err_joint_angles(0, 0));
        arm1.setAngleRad(min_err_joint_angles(1, 0));
        arm2.setAngleRad(min_err_joint_angles(2, 0));

        base.print();
        arm1.print();
        arm2.print();

        Q_prev = Q_star;

        sleep_ms(1000);
    }
	
	return 0;
}

