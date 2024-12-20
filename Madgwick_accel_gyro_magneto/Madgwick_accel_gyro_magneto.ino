#include <SparkFunMPU9250-DMP.h>
#include <math.h>

#define SerialPort SerialUSB

MPU9250_DMP imu;

//Quaternion
float q[4] = {0.1f, 0.0f, 0.0f, 0.0f};
float q1 = q[0], q2 = q[1], q3 = q[2], q4 =q[3];
float bx, bz = 0;
float gyr_bias[3] = {0.0f, 0.0f, 0.0f};
float beta = 0.0003; //0.099|0.001|0.01
float zeta = 0.00001; //442|0.000001|0.001

//delta t
float prevTime, prevTime1, currTime = 0;
float dt;
int n = 1;
int m = 1;

//kalibrasi
bool magneto_calibration = true;
bool magneto_timer_activation = true;
float max_magX, min_magX, max_magY, min_magY, max_magZ, min_magZ;
float offset_x, offset_y,offset_z;
float avg_delta_x, avg_delta_y, avg_delta_z, avg_delta;
float scale_x, scale_y, scale_z;

bool gyroscope_calibration = true;
bool gyroscope_timer_activation =true;
float avr_gyroX, avr_gyroY, avr_gyroZ;
float offset_gyroX, offset_gyroY,offset_gyroZ;
int sample = 1;

void setup() 
{
    SerialPort.begin(115200);
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }


  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);  
  imu.setGyroFSR(1000);// Gyro options are +/- 250, 500, 1000, or 2000 dps  
  imu.setAccelFSR(4);// Accel options are +/- 2, 4, 8, or 16 g 
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)
  imu.setLPF(10); // Can be any of the following: 188, 98, 42, 20, 10, 5
  imu.setSampleRate(50); // Acceptable values range from 4Hz to 1kHz
  imu.setCompassSampleRate(50); // This value can range between: 1-100Hz

  prevTime = millis();
  delay(3000);
}

void loop() 
{
  if ( imu.dataReady() )
  {

    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    float accelX = imu.calcAccel(imu.ax);
    float accelY = imu.calcAccel(imu.ay);
    float accelZ = imu.calcAccel(imu.az);

    float gyroX = imu.calcGyro(imu.gx);
    float gyroY = imu.calcGyro(imu.gy);
    float gyroZ = imu.calcGyro(imu.gz);

    float magX = imu.calcMag(imu.mx);
    float magY = imu.calcMag(imu.my);
    float magZ = imu.calcMag(imu.mz);

    while(gyroscope_calibration){
      if ( imu.dataReady() )
      {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        //Mengaktifkan timer untuk lama kalibrasi
        if(gyroscope_timer_activation){
          prevTime = millis();
          gyroscope_timer_activation = false;
        }

        //Menampilkan nilai awal 
        if(n == 1){         
          SerialPort.print("Gyro X: ");SerialPort.println(gyroX);
          SerialPort.print("Gyro Y: ");SerialPort.println(gyroY);
          SerialPort.print("Gyro Z: ");SerialPort.println(gyroZ);

          SerialPort.println("Gyroscope calibration start...");
          n = 0;
        }

        //Melakukan pembacaan nilai sensor
        float gyroX = imu.calcGyro(imu.gx);
        float gyroY = imu.calcGyro(imu.gy);
        float gyroZ = imu.calcGyro(imu.gz);

        //Mencari nilai rata-rata dari setiap sumbu
        avr_gyroX = avr_gyroX + gyroX;
        avr_gyroY = avr_gyroY + gyroY;
        avr_gyroZ = avr_gyroZ + gyroZ;

        //Mencari nilai offset
        offset_gyroX = avr_gyroX/sample;
        offset_gyroY = avr_gyroY/sample;
        offset_gyroZ = avr_gyroZ/sample;
        
        //Menampilkan hasil kalibrasi jika sudah selesai
        if(millis()-prevTime >= 15000){
          SerialPort.println("Gyroscope Calibration Finished");
          SerialPort.print("Sample: ");SerialPort.println(sample);
          SerialPort.print("Average X: ");SerialPort.println(avr_gyroX);
          SerialPort.print("Average Y: ");SerialPort.println(avr_gyroY);
          SerialPort.print("Average Z: ");SerialPort.println(avr_gyroZ);
          SerialPort.print("Offset X: ");SerialPort.println(offset_gyroX);
          SerialPort.print("Offset Y: ");SerialPort.println(offset_gyroY);
          SerialPort.print("Offset Z: ");SerialPort.println(offset_gyroZ);
          SerialPort.print("Gyro X: ");SerialPort.println(gyroX-offset_gyroX);
          SerialPort.print("Gyro Y: ");SerialPort.println(gyroY-offset_gyroY);
          SerialPort.print("Gyro Z: ");SerialPort.println(gyroZ-offset_gyroY);
          gyroscope_calibration = false;
        }
        sample = sample + 1;
        }    
    }


    while(magneto_calibration){
      if (imu.dataReady() )
      {
        imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
        //Mengaktifkan timer untuk kalibrasi
        if(magneto_timer_activation){
          prevTime = millis();          
          magneto_timer_activation = false;
        }

        //Menampilkan nilai awal
        if(m == 1){
          max_magX = magX;
          min_magX = magX;
          max_magY = magY;
          min_magY = magY;
          max_magZ = magZ;
          min_magZ = magZ;
          SerialPort.print("Max X: ");SerialPort.print(max_magX);SerialPort.print("\t");SerialPort.print("  Min X: ");SerialPort.println(min_magX);
          SerialPort.print("Max Y: ");SerialPort.print(max_magY);SerialPort.print("\t");SerialPort.print("  Min Y: ");SerialPort.println(min_magY);
          SerialPort.print("Max Z: ");SerialPort.print(max_magZ);SerialPort.print("\t");SerialPort.print("  Min Z: ");SerialPort.println(min_magZ);

          SerialPort.println("Magnetometer calibration start...");
          m = 0;
        }

        //Membaca nilai sensor
        float magX = imu.calcMag(imu.mx);
        float magY = imu.calcMag(imu.my);
        float magZ = imu.calcMag(imu.mz);
        
        //Mencari nilai max dan mix untuk setiap sumbu
        if(magX>max_magX){
          max_magX = magX;      
        }
        if(magY>max_magY){
          max_magY = magY;      
        }
        if(magZ>max_magZ){
          max_magZ = magZ;      
        }

        if(magX<min_magX){
          min_magX = magX;      
        }
        if(magY<min_magY){
          min_magY = magY;      
        }
        if(magZ<min_magZ){
          min_magZ = magZ;      
        }

        //hard iron
        offset_x = (max_magX + min_magX)/2;
        offset_y = (max_magY + min_magY)/2;
        offset_z = (max_magZ + min_magZ)/2;

        //soft iron
        avg_delta_x = (max_magX - min_magX)/2;
        avg_delta_y = (max_magY - min_magY)/2;
        avg_delta_z = (max_magZ - min_magZ)/2;

        avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z)/3;

        //Mencari skala
        scale_x = avg_delta/avg_delta_x;
        scale_y = avg_delta/avg_delta_y;
        scale_z = avg_delta/avg_delta_z;
        
        //Menampilkan hasil kalibrasi
        if(millis()-prevTime >= 15000){
          SerialPort.println("Magnetometer Calibration Finished");
          SerialPort.print("avg_delta: ");SerialPort.println(avg_delta);
          SerialPort.print("Max X: ");SerialPort.print(max_magX);SerialPort.print("\t");SerialPort.print("Min X: ");SerialPort.println(min_magX);
          SerialPort.print("Max Y: ");SerialPort.print(max_magY);SerialPort.print("\t");SerialPort.print("Min Y: ");SerialPort.println(min_magY);
          SerialPort.print("Max Z: ");SerialPort.print(max_magZ);SerialPort.print("\t");SerialPort.print("Min Z: ");SerialPort.println(min_magZ);
          SerialPort.print("Offset X: ");SerialPort.println(offset_x);
          SerialPort.print("Offset Y: ");SerialPort.println(offset_y);
          SerialPort.print("Offset Z: ");SerialPort.println(offset_z);
          SerialPort.print("Scale X: ");SerialPort.println(scale_x);
          SerialPort.print("Scale Y: ");SerialPort.println(scale_y);
          SerialPort.print("Scale Z: ");SerialPort.println(scale_z);
          magneto_calibration = false;
          delay(3000);
        }
      }
    }
    

    float ax = accelX, 
          ay = accelY, 
          az = accelZ;

    //Gyro Calibration
    float offset_gyroX = -0.278;
    float offset_gyroY = 1.025;
    float offset_gyroZ = 1.06;
    float gx = (gyroX-offset_gyroX)*(PI/180),
          gy = (gyroY-offset_gyroY)*(PI/180),
          gz = (gyroZ-offset_gyroZ)*(PI/180);

    //Magneto Calibration
    float offset_x = 55.073;
    float offset_y = 5.18;
    float offset_z = -99.986;
    float scale_x = 1.005;
    float scale_y = 1.0516;
    float scale_z = 0.9516;
    float mx = (magX-offset_x)*scale_x,
          my = (magY-offset_y)*scale_y,
          mz = (magZ-offset_z)*scale_z;

    
    //delta t
    currTime = millis();
    dt = (currTime-prevTime)/1000;
    prevTime = currTime;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // //MENGHITUNG NILAI FLUX IN THE EART FRAME madgwick
    // float h_x = 2*mx*(0.5-q3*q3-q4*q4)+2*my*(q2*q3-q1*q4)    +2*mz*(q2*q4+q1*q3);
    // float h_y = 2*mx*(q2*q3+q1*q4)    +2*my*(0.5-q2*q2-q4*q4)+2*mz*(q3*q4-q1*q2);
    // float h_z = 2*mx*(q2*q4-q1*q3)    +2*my*(q3*q4+q1*q2)    +2*mz*(0.5-q2*q2-q3*q3); 
    // float b_x = sqrt((h_x * h_x) + (h_y * h_y));
    // float b_z = h_z;

    //MENGHITUNG NILAI FLUX IN THE EART FRAME kriswiner
    float h_x = mx*q1*q1 - 2*q1*my*q4 + 2*q1*mz*q3 + mx*q2*q2 + 2*q2*my*q3 + 2*q2*mz*q4 - mx*q3*q3 - mx*q4*q4;
    float h_y = 2*q1*mx*q4 + my*q1*q1 - 2*q1*mz*q2 + 2*q2*mx*q3 - my*q2*q2 + my*q2*q2 + my*q3*q3 + 2*q3*mz*q4 - my*q4*q4;
    float b_x = (sqrt((h_x * h_x) + (h_y * h_y)))/2;
    float b_z = (-2*q1*mx*q3 +2*q1*my + mz*q1*q1 + 2*q2*mx*q4 - mz*q2*q2+ 2*q3*my*q4 - mz*q3*q3 + mz*q4*q4)/2;

    

    //FUNGSI OBJEKTIF
    //Pada perhitungan ini nilai q1,q2,q3,q4 merupakan nilai
    //saat object berada pada posisi diam, yaitu q = [1,0,0,0].
    //Kemudian ax, ay, az, mx, my, mz merupakan raw data dari pengukuran
    //accelerometer pada setiap sumbu. Selain itu bx dan bz merupakan
    //bagian dari reference direction of the earth's magnetic field (pers. 46)
    //          |         2(q2q4-q1q3) -     ax       |
    //          |         2(q1q2+q3q4) -     ay       |
    // fg,b=    |         2(0.5-q2q2-q3q3) - az       |
    //          |2bx(0.5-q3q3-q4q4)+2bz(q2q4-q1q3) -mx|
    //          |2bx(q2q3-q1q4)+2bz(q1q2+q3q4)     -my|
    //          |2bx(q1q3+q2q4)+2bz(0.5-q2q2-q3q3) -mz|

    
    
    float obj_f[6] = {
      2*(q2*q4 - q1*q3)  -ax,
      2*(q1*q2 - q3*q4)  -ay,
      2*(0.5-q2*q2-q3*q3)-az,
      2*bx*(0.5-q3*q3-q4*q4)+ 2*bz*(q2*q4-q1*q3)     - mx,
      2*bx*(q2*q3-q1*q4)    + 2*bz*(q1*q2+q3*q4)     - my,
      2*bx*(q1*q3+q2*q4)    + 2*bz*(0.5-q2*q2-q3*q3) - mz
    }; 

    //FUNGSI JACOBIAN
    //Untuk nilai yang digunakan pada jacobian tranpose sama dengan nilai
    //yang digunakan pada 
    //        |-2q3    2q4    0.0f          -2bzq3    -2bxq4+2bzq2            2bxq3|
    //Jg,b =  | 2q4    2q1    -4q2           2bzq4     2bxq3+2bzq1      2bzq4-4bzq2|
    //        |-2q1    2q4    -4q3    -4bxq3-2bzq1     2bxq2+2bzq4      2bxq1-4bzq3| 
    //        | 2q2    2q3    0.0f    -4bxq4+2bzq2    -2bxq1+2bzq3            2bxq2|     

    float jac_tr[24] = {
      -2*q3, 2*q2,  0.0f,         -2*bz*q3, -2*bx*q4+2*bz*q2,         2*bx*q3,
       2*q4, 2*q1, -4*q2,          2*bz*q4,  2*bx*q3+2*bz*q1, 2*bx*q4-4*bz*q2,
      -2*q1, 2*q4, -4*q3, -4*bx*q3-2*bz*q1,  2*bx*q2+2*bz*q4, 2*bx*q1-4*bz*q3,
       2*q2, 2*q3,  0.0f, -4*bx*q4+2*bz*q2, -2*bx*q1+2*bz*q3,         2*bx*q2
    };  

      

    //GRADIENT DESCENT
    //Gradien descent menghasilkan nilai berupa rate of orientation
    //sensor dengan lambang q(dot). q(dot memiliki) satuan rad/s.
    //                                             |f0|
    //                   |j0  j1  j2  j3   j4  j5 ||f1|
    // q_dot_hat (qdh) = |j6  j7  j8  j9   j10 j11||f2|
    //                   |j12 j13 j14 j15  j16 j17||f3|
    //                   |j18 j19 j20 j21  j22 j23||f4|
    //                                             |f5|
    //                                     
    //           |[j0] [f0] [j1] [f1] [j2] [f2] [j3] [f3]  [j4] [f4] [j5] [f5]|
    // delta f = |[j6] [f0] [j7] [f1] [j8] [f2] [j9] [f3]  [j10][f4] [j11][f5]|
    //           |[j12][f0] [j13][f1] [j14][f2] [j15][f3]  [j16][f4] [j17][f5]|
    //           |[j18][f0] [j19][f1] [j20][f2] [j21][f3]  [j22][f4] [j23][f5]|
    //                                     


    float qdh[4] = {
      jac_tr[0] *obj_f[0] + jac_tr[1] *obj_f[1] + jac_tr[2] *obj_f[2] + jac_tr[3] *obj_f[3] + jac_tr[4] *obj_f[4] + jac_tr[5] *obj_f[5],
      jac_tr[6] *obj_f[0] + jac_tr[7] *obj_f[1] + jac_tr[8] *obj_f[2] + jac_tr[9] *obj_f[3] + jac_tr[10]*obj_f[4] + jac_tr[11]*obj_f[5],
      jac_tr[12]*obj_f[0] + jac_tr[13]*obj_f[1] + jac_tr[14]*obj_f[2] + jac_tr[15]*obj_f[3] + jac_tr[16]*obj_f[4] + jac_tr[17]*obj_f[5],
      jac_tr[18]*obj_f[0] + jac_tr[19]*obj_f[1] + jac_tr[20]*obj_f[2] + jac_tr[21]*obj_f[3] + jac_tr[22]*obj_f[4] + jac_tr[23]*obj_f[5]
    };


    // //NORMALISASI
    //Karena sudah dilakukan normalisasi, unit rate of orientation
    //of sensor yang dihitung menggunakan gradient descent tidak memiliki 
    //satuan lagi
    float q_dot_hat_norm = sqrt(qdh[0]*qdh[0] +
                                qdh[1]*qdh[1] +
                                qdh[2]*qdh[2] +
                                qdh[3]*qdh[3]);

    qdh[0] = qdh[0]/q_dot_hat_norm;
    qdh[1] = qdh[1]/q_dot_hat_norm;
    qdh[2] = qdh[2]/q_dot_hat_norm;
    qdh[3] = qdh[3]/q_dot_hat_norm;

    //GYROSCOPE ERROR
    //Pada perhitungan gyroscope error atau gyroscope bias menggunakan
    //nilai quaternion conjugate di kalikan menggunakan perkalian quaternion
    //dengan q(hat)(dot). Operasi yang digunakan di bawah ini merupakan rumus
    //dari laporan madwick. Untuk nilai skalar pada operasi ini tidak digunakan
    //sehingga hanya ada 3 baris saja.
    float gyr_error[3] = {
      2*q1*qdh[1] - 2*q2*qdh[0] - 2*q3*qdh[3] + 2*q4*qdh[2],
      2*q1*qdh[2] + 2*q2*qdh[3] - 2*q3*qdh[0] - 2*q4*qdh[1],
      2*q1*qdh[3] - 2*q2*qdh[2] + 2*q3*qdh[1] - 2*q4*qdh[0]
    };

    //MENGHITUNG BIAS
    //Menghitung bias ini dilakukukan dengan mencari nilai akumulasi
    //dengan cara mengalikan nilai gyr_error dengan nilai zeta dan 
    //delta t
    gyr_bias[0] = gyr_bias[0] + gyr_error[0]*dt*zeta;
    gyr_bias[1] = gyr_bias[1] + gyr_error[1]*dt*zeta;
    gyr_bias[2] = gyr_bias[2] + gyr_error[2]*dt*zeta;


    //MENGHAPUS NILAI BIAS
    //Dengan menghapus nilai bias pada gyroscope
    gx =  gx - gyr_bias[0];
    gy =  gy - gyr_bias[1];
    gz =  gz - gyr_bias[2];

    // //MENGHITUNG NILAI TURUNAN QUATERNION
    //Nilai turunan dari quaternion ini memiliki lambang q dot
    //dengan satuan rad/s. Kemudian normalisasi ti selalu bernilai 1
    //q_dot = 0.5q_hat ⨂ 𝜔 (pers. 12)
    //             |-q2*gx - q3*gy - q4*gz|
    // q_dot = 0.5*| q1*gx + q3*q3 - q4*gz|
    //             | q1*gy - q2*gz + q4*gx|
    //             | q1*gz + q2*gy - q3*gx|
    float q_dot[4] = {
      -0.5*q2*gx-0.5*q3*gy-0.5*q4*gz,
      0.5*q1*gx+0.5*q3*gz-0.5*q4*gy,
      0.5*q1*gy-0.5*q2*gz+0.5*q4*gx,
      0.5*q1*gz+0.5*q2*gy-0.5*q3*gx
    };

    q_dot[0] = q_dot[0] - beta*qdh[1];
    q_dot[1] = q_dot[1] - beta*qdh[2];
    q_dot[2] = q_dot[2] - beta*qdh[3];
    q_dot[3] = q_dot[3] - beta*qdh[4];


    //Ingegrate Rate of Rotation of Sensor from Gyroscope
    q1 = q1 + q_dot[0]*dt;
    q2 = q2 + q_dot[1]*dt;
    q3 = q3 + q_dot[2]*dt;
    q4 = q4 + q_dot[3]*dt;

    
    norm = sqrt(q1*q1+q2*q2+q3*q3+q4*q4);
    norm = 1.0f/norm;
    q1 = q1*norm;
    q2 = q2*norm;
    q3 = q3*norm;
    q4 = q4*norm;

    SerialPort.print("w");
    SerialPort.print(q1);
    SerialPort.print("wa");
    SerialPort.print(q2);
    SerialPort.print("ab");
    SerialPort.print(q3);
    SerialPort.print("bc");
    SerialPort.print(q4);
    SerialPort.println("c");
  }

  
}






