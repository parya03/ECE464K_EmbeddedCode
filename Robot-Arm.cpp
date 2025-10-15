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
    200, // max number of iterations
    quik::ALGORITHM_QUIK, // algorithm (ALGORITHM_QUIK, ALGORITHM_NR or ALGORITHM_BFGS)
    1e-12, // Exit tolerance
    1e-14, // Minimum step tolerance
    0.05, // iteration-to-iteration improvement tolerance (0.05 = 5% relative improvement)
    20, // max consequitive gradient fails
    80, // Max gradient fails
    1e-10, // lambda2 (lambda^2, the damping parameter for DQuIK and DNR)
    0.34, // Max linear error step
    1 // Max angular error step
);

bool run_state = true;

// // Used to read input data
// void uart_rx_interrupt() {
//     while (uart_is_readable(uart0)) {
//         uint8_t ch = uart_getc(uart0);

//         if(ch == '0') {
//             run_state = false;
//         }
//         if(ch == '1') {
//             run_state = true;
//         }
//         // // Can we send it back?
//         // if (uart_is_writable(uart0)) {
//         //     // Change it slightly first!
//         //     ch++;
//         //     uart_putc(uart0, ch);
//         // }
//         // chars_rxed++;
//     }
// }

int main() {
    stdio_init_all();

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(0, GPIO_FUNC_PWM);
    // gpio_set_function(1, GPIO_FUNC_PWM);

    // gpio_init(2);
    // gpio_set_pulls(2, false, true);
    // gpio_set_dir(2, GPIO_IN);

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

    Servo base(2, 45);
    Servo arm1(3);
    Servo arm2(4);

    base.startPWMControllers();
    arm1.startPWMControllers();
    arm2.startPWMControllers();

    // Set up a UART RX interrupt
    // And set up and enable the interrupt handlers
    // irq_set_exclusive_handler(UART0_IRQ, uart_rx_interrupt);
    // irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    // uart_set_irq_enables(uart0, true, false);
    
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
        
        Matrix<float,4,4>    Tn(4,4); 	    // 4N * 4 matrix of vertically stacked transforms to be solved.
                                                    // This is just a convenient way of sending in an array of transforms.
        
        // Generate some random joint configurations for the robot
        Q.setRandom(DOF, N);

        // Perturb true answers slightly to get initial "guess" (knock over by 0.1 radians)
        Q0 = Q.array() + 0.1;
        
        // Do forward kinematics of each Q sample and store in the "tall" matrix
        // of transforms, Tn
        // for (int i = 0; i < N; i++){
        //     R->FKn( Q.col(i), T );
        //     Tn.middleRows<4>(i*4) = T;
        // }

        Tn << 0, 0, 0, 5, \
                0, 0, 0, 5, \
                0, 0, 0, 1, \
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

        printf("The final joint angles are:\n");
        for (int i = 0; i < Q_star.rows(); i++) {
            for (int j = 0; j < Q_star.cols(); j++) {
                printf("%f = %f Pi", Q_star(i, j), Q_star(i, j)/float(M_PI));
            }
            printf("\n");
        }
        printf("\n");

        // printf("Final normed error is:\n");
        // for (int j = 0; j < e_star.cols(); j++) {
        //     float normed_error = 0.0;
        //     for (int i = 0; i < e_star.rows(); i++) {
        //         normed_error += e_star(i, j) * e_star(i, j);
        //     }
        //     printf("%f ", sqrt(normed_error));
        // }
        // printf("\n\n");

        // printf("Break reason is:\n");
        // for (const auto& reason : breakReason) {
        //     printf("%d ", reason);
        // }
        // printf("\n");

        // printf("Number of iterations:\n");
        // for (const auto& iter_i : iter) {
        //     printf("%d ", iter_i);
        // }
        // printf("\n");
        
        printf("Commanded transform (Tn):\n");
        for (int i = 0; i < Tn.rows(); i++) {
            for (int j = 0; j < Tn.cols(); j++) {
                printf("%f ", Tn(i, j));
            }
            printf("\n");
        }
        printf("IK finished!\n");
        printf("Total time taken: %d ms\n", endTime - startTime);

        base.setAngleRad(Q_star(0, 0));
        arm1.setAngleRad(Q_star(1, 0));
        arm2.setAngleRad(Q_star(2, 0));
        base.print();
        arm1.print();
        arm2.print();
    }
	
	return 0;
}

