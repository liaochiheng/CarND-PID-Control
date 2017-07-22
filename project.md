# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

1. Describe the effect each of the P, I, D components had in your implementation.
  * Effect of P: The bigger the P is, more fast the oscillations is.
  * Effect of I: The I is to balance the bias.
  * Effect of D: The D is to make oscillations converge.

2. Describe how the final hyperparameters were chosen.
  * I used twiddle to find the best hyperparameters.
  * In `PID.cpp(line 45)`, there is a function `PID::Twiddle_Init`, which initialize twiddle including several variables. The first parameter `tw_switch=true/false` means whether twiddle or not.
    ```
      pid.Twiddle_Init(true, 1300);
    ```
  * If `tw_switch = true`(twiddle mode), the function `PID::Twiddle` will twiddle for the best parameters, and begin with :
    ```
      twiddle_p[0] = 0.2;
      twiddle_p[1] = 3.0;
      twiddle_p[2] = 0.01;
    ```
    and with dp:
    ```
    twiddle_dp[0] = 0.1;
    twiddle_dp[1] = 1.0;
    twiddle_dp[2] = 0.005;
    ```
  * In twiddle mode, the program will reset when the car is off road by checking cte with `cte < -4.0 || cte > 4.0(PID.cpp line 75)`.
  * In twiddle mode, the program will print out current `Kp, Kd, Ki` if `err` is better (PID.cpp line 103).
    ```
    Twiddle # 79 ==> 1.0987, 7.41101, 0.00467523  best_err = 0.0375717
    ```
  * In twiddle mode, the program will also print out current `Kp, Kd, Ki` if `err` is no better (PID.cpp line 133).
    ```
    Twiddle # 80 [No-Better]: 1.0987, 7.41101, 0.0065548  err = 0.0494153
    ```
  * In twiddle mode, the program will print out `twiddle_dp` every 10 runs (PID.cpp line 157):
    ```
    =======================
    Twiddle dp: 0.0672681, 0.450307, 0.00123318
    =======================
    ```
  * In twiddle mode, if `sum of twiddle_dp` is small enough, such as `sum of twiddle_dp < 0.01 (PID.cpp line 141)`, that would be the best parameters, and the twiddle mode will stop. Then the best prameter will be set and print out the best parameters.
    ```
        std::cout << "********************************************************" << endl;
        std::cout << "Twiddle End. The best p = [" << twiddle_p_best[0] << ", "
              << twiddle_p_best[1] << ", " << twiddle_p_best[2] << "]." << endl;
        std::cout << "The best error is: " << twiddle_err_best << endl;
        std::cout << "********************************************************" << endl;
    ```
  * Practically, the `sum of twiddle_dp` never got that small(0.01), and only converged to about 0.37 after few hours.
  * To see all the print-out, [click here to check out the `log` file](twiddle-output.log)
  * After twiddle, i use the best hyperparameters to init pid, then the program will work with self-driving mode. (main.cpp line 47):
    ```
    pid.Init(1.0987, 7.41101, 0.00467523);
    ```
