/**
 * Inverse Kinematics Demo, Walter Gordy 2014, 2023
 */
float[][][] A, T; // matrix types

float theta1=0, theta2=0, theta3=0, theta4=0, theta5=0, theta6=0;

// Arm lengths and tool parameters
float a2 = 22.0;
float a3 = 0;
float d3 = 0;
float d4 = 17.0;
float tool = 15.5;

// Position parameters
float px, py, pz;

// Rotation matrix elements
float r11 = 1, r12 = 0, r13 = 0;
float r21 = 0, r22 = 1, r23 = 0;
float r31 = 0, r32 = 0, r33 = 1;

// Homogenous position parameters
float hpx = 0, hpy = -15, hpz = 20;

// Rotation angles
float rx = 0, ry = PI, rz = 0;

// Previous homogenous position parameters
float phpx = hpx, phpy = hpy, phpz = hpz;

// Previous rotation angles
float prx = rx, pry = ry, prz = rz;

void setup()
{
  A = new float[7][4][4];
  T = new float[7][4][4];
  size(850, 480, P3D);
  for (int i = 0; i < 7; i++)
    A[i] = matIdentity(4); // create new 4x4 matrix
}

void draw()
{
  background(127);

  stroke(255);
  strokeWeight(0.1);

  if (mousePressed)
  {

    rx += (pmouseX - mouseX) * PI / width;
    ry += (pmouseY - mouseY) * PI / height;
    //rz += sliderRZ.getValue()/(PI*1000);
  } else
  {
    hpx = -(mouseX - width/2) * 30.0 / width;
    hpz = (height - mouseY) * 30.0 / height;
    //hpz -= sliderZ.getValue()/1000;
  }

  //println("draw2");
  if (calculateIK() == false)
  {
    hpx = phpx;
    hpy = phpy;
    hpz = phpz;

    rx = prx;
    ry = pry;
    rz = prz;
    calculateIK();
  } else
  {

    phpx = hpx;
    phpy = hpy;
    phpz = hpz;

    prx = rx;
    pry = ry;
    prz = rz;
  }

  perspective();

  translate(width/2, height- height/3, 0);

  rotateX(PI/2);

  pushMatrix();
  scale(10);

  translate(-hpx, -hpy, hpz);
  fill(255, 0, 0);
  stroke(255, 0, 0);
  sphere(0.1);
  popMatrix();
  fill(255, 255, 255);

  pushMatrix();
  scale(10);
  stroke(0);
  fill(255);
  applyDHMatrix(A[0]);
  box(6, 6, 1);
  applyDHMatrix(A[1]);

  pushMatrix();
  translate((a2)/2, 0, 0);
  box(a2-5, 2, 1);
  popMatrix();

  box(1, 1, 3);
  applyDHMatrix(A[2]);
  box(1, 1, 3);

  pushMatrix();
  translate(0, d4/2, 0);
  box(2, d4-5, 1);
  popMatrix();

  applyDHMatrix(A[3]);
  box(1, 1, 3);
  applyDHMatrix(A[4]);
  box(1, 1, 3);
  applyDHMatrix(A[5]);
  //  box(1,1,3);

  pushMatrix();
  translate(0, 0, tool/2);
  box(1, 1, tool-5);
  popMatrix();

  applyDHMatrix(A[6]);
  fill(255, 0, 0);
  box(3, 6, 0.5);

  popMatrix();
  stroke(255);
}

boolean calculateIK()
{
  //println("calc1");

  float[][] rotation = rotationMatrix(rx, ry, rz);


  r11 = rotation[0][0];
  r12 = rotation[0][1];
  r13 = rotation[0][2];
  r21 = rotation[1][0];
  r22 = rotation[1][1];
  r23 = rotation[1][2];
  r31 = rotation[2][0];
  r32 = rotation[2][1];
  r33 = rotation[2][2];

  px = hpx + tool * r13;
  py = hpy + tool * r23;
  pz = hpz - tool * r33;


  theta1 = atan2(py, px) - atan2(d3, sqrt(px*px + py*py - d3*d3));

  float K = (px*px + py*py + pz*pz - a2*a2 - a3*a3 - d3*d3 - d4*d4) / (2 * a2);

  theta3 = atan2(a3, d4) - atan2(K, -sqrt(a3*a3 + d4*d4 - K*K));

  float theta23 = atan2((-a3-a2*cos(theta3))*pz - (cos(theta1)*px + sin(theta1)*py)*(a2*sin(theta3) - d4),
    (a2*sin(theta3)-d4)*pz - (a3+a2*cos(theta3))*(cos(theta1)*px + sin(theta1)*py));
  theta2 = theta23 - theta3;

  theta4 = atan2(-r13*sin(theta1)+r23*cos(theta1), -r13*cos(theta1)*cos(theta2+theta3) - r23*sin(theta1)*cos(theta2 + theta3) + r33*sin(theta2 + theta3));

  float s5 = -(r13*(cos(theta1)*cos(theta2 + theta3)*cos(theta4) +sin(theta1)*sin(theta4)) + r23*(sin(theta1)*cos(theta2+theta3)*cos(theta4) - cos(theta1)*sin(theta4)) - r33*(sin(theta2 + theta3)*cos(theta4)));
  float c5 = r13*(-cos(theta1)*sin(theta2 + theta3)) + r23*(-sin(theta1)*sin(theta2+theta3)) + r33*(-cos(theta2 + theta3));

  theta5 = atan2(s5, c5);

  float s6 = -r11*(cos(theta1)*cos(theta2 + theta3)*sin(theta4) - sin(theta1)*cos(theta4)) - r21*(sin(theta1)*cos(theta2 + theta3)*sin(theta4) + cos(theta1)*cos(theta4)) + r31*(sin(theta2 + theta3)*sin(theta4));
  float c6 = r11*((cos(theta1)*cos(theta2+theta3)*cos(theta4) + sin(theta1)*sin(theta4))*cos(theta5) - cos(theta1)*sin(theta2+theta3)*sin(theta5)) + r21*((sin(theta1)*cos(theta2+theta3)*cos(theta4) - cos(theta1)*sin(theta4))*cos(theta5) - sin(theta1)*sin(theta2+theta3)*sin(theta5))-r31*(sin(theta2+theta3)*cos(theta4)*cos(theta5) + cos(theta2+theta3)*sin(theta5));

  theta6 = atan2(s6, c6);

  // //println(px + "\t\t" + py + "\t\t" + pz + "\t\t" + atan2(d3,sqrt(px*px + py*py - d3*d3)));
  //println("calc2");
  A[0] = DHMatrix(A[0], 0, 0, 0, theta1);     // calculate initial DH Matrix
  A[1] = DHMatrix(A[1], 0, -PI/2, 0, theta2);
  A[2] = DHMatrix(A[2], a2, 0, 0, theta3);
  A[3] = DHMatrix(A[3], 0, -PI/2, d4, theta4);
  A[4] = DHMatrix(A[4], 0, PI/2, 0, theta5);
  A[5] = DHMatrix(A[5], 0, -PI/2, 0, theta6);
  A[6] = DHMatrix(A[6], 0, 0, tool, 0);

  T[0]  = A[0];                    // Calculate Positions
  T[1] = matMultiply(T[0], A[1]);
  T[2] = matMultiply(T[1], A[2]);
  T[3] = matMultiply(T[2], A[3]);
  T[4] = matMultiply(T[3], A[4]);
  T[5] = matMultiply(T[4], A[5]);
  T[6] = matMultiply(T[5], A[6]);
  //println("calc3");

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (T[6][i][j] == Float.NaN )
      {
        return false;
      }
    }
  }

  return true;
}

float[][] DHMatrix(float[][] Ai, float a_, float alpha_, float d_, float theta_ )
{
  Ai[0][0] = cos(theta_);
  Ai[0][1] = -sin(theta_);
  Ai[0][2] = 0;
  Ai[0][3] = a_ ;

  Ai[1][0] = sin(theta_) * cos(alpha_);
  Ai[1][1] = cos(theta_) * cos(alpha_);
  Ai[1][2] = -sin(alpha_);
  Ai[1][3] = -sin(alpha_) * d_;

  Ai[2][0] = sin(theta_) * sin(alpha_);
  Ai[2][1] = cos(theta_) * sin(alpha_);
  Ai[2][2] = cos(alpha_);
  Ai[2][3] = cos(alpha_) * d_;

  Ai[3][0] = 0;
  Ai[3][1] = 0;
  Ai[3][2] = 0;
  Ai[3][3] = 1;

  return Ai;
}

void applyDHMatrix(float[][] Ai)
{
  applyMatrix(Ai[0][0], Ai[0][1], Ai[0][2], Ai[0][3],
    Ai[1][0], Ai[1][1], Ai[1][2], Ai[1][3],
    Ai[2][0], Ai[2][1], Ai[2][2], Ai[2][3],
    Ai[3][0], Ai[3][1], Ai[3][2], Ai[3][3]);
}

float[][] rotationMatrix(float rx, float ry, float rz)
{
  float[][] Rx = matIdentity(3);
  float[][] Ry = matIdentity(3);
  float[][] Rz = matIdentity(3);

  Rx[1][1] = cos(rx);
  Rx[1][2] = -sin(rx);
  Rx[2][2] = cos(rx);
  Rx[2][1] = sin(rx);

  Ry[0][0] = cos(ry);
  Ry[2][2] = cos(ry);
  Ry[2][0] = -sin(ry);
  Ry[0][2] = sin(ry);

  Rz[0][0] = cos(rz);
  Rz[1][1] = cos(rz);
  Rz[0][1] = -sin(rz);
  Rz[1][0] = sin(rz);

  return matMultiply(Rx, matMultiply(Ry, Rz));
}

float[][] matMultiply(float a[][], float b[][]) {//a[m][n], b[n][p]
  if (a.length == 0) return new float[0][0];
  if (a[0].length != b.length) return null; //invalid dims

  int n = a[0].length;
  int m = a.length;
  int p = b[0].length;

  float ans[][] = new float[m][p];

  for (int i = 0; i < m; i++) {
    for (int j = 0; j < p; j++) {
      for (int k = 0; k < n; k++) {
        ans[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return ans;
}

float[][] matIdentity(int n) {
  float[][] mat = new float[n][n];

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      mat[i][j] = (i == j) ? 1.0 : 0.0;  // 1 on diagonal, 0 otherwise
    }
  }

  return mat;
}
