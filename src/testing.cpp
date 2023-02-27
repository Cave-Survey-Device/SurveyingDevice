// #include <Arduino.h>

// void test_lasercalibration(){
//   char buffer[150];

//   Serial.begin(9600);
//   Serial.println("");
//   Serial.println("Beginning!");
//   MatrixXf g_vec(3,9);
//   g_vec <<  9.25185853854297e-18, 0.3420201433256687, 0.24184476264797528, 3.019455222692793e-17, -0.24184476264797522, -0.3420201433256687, -0.2418447626479753, -5.35762225266119e-17, 0.2418447626479752,
//             0.0, 0.0, 0.7071067811865476, 1.0, 0.7071067811865476, 1.2246467991473532e-16, -0.7071067811865475, -1.0, -0.7071067811865477,
//             3.700743415417188e-17, 0.9396926207859084, 0.6644630243886748, 9.454701216556439e-17, -0.6644630243886747, -0.9396926207859084, -0.6644630243886749, -1.3561129988000566e-16, 0.6644630243886746;
//   // g_vec << 1.0,2.0,3.0,
//   //           4.0,5.0,6.0,
//   //           7.0,8.0,9.0;
//   Serial.println("SVD time!");
//   Vector3f normal = calc_SVD(g_vec);

//   Serial.println("Initialising true_vec vars...");
//   float disto_len = 0.1;
//   VectorXd laser_distances(3);
//   laser_distances << 20,21,19;

//   Serial.println("Beginning true_vec calcs...");
//   Vector3f true_vec = calc_true_vec(normal, laser_distances, disto_len);

//   sprintf(buffer, "\nX: %f \nY: %f \nZ: %f\n", true_vec[0], true_vec[1], true_vec[2]);
//   Serial.printf(buffer);
// }