/**
 * Inverse Kinematics Demo, Walter Gordy 2014
*/
float[][] A1,A2,A3,A4,A5,A6,A7,T1,T2,T3,T4,T5,T6,T7; // matrix types

float theta1=0,theta2=0,theta3=0,theta4=0,theta5=0,theta6=0;

float a2 = 22.0;
float a3 = 0;
float d3 = 0;
float d4 = 17.0;
float tool = 15.5;
float px,py,pz;
float r11=1,r12=0,r13=0;
float r21=0,r22=1,r23=0;
float r31=0,r32=0,r33=1;

float hpx=0,hpy=-15,hpz=20;
float rx=0,ry=PI,rz=0;

float phpx = hpx;
float phpy = hpy;
float phpz = hpz;

float prx = rx;
float pry = ry;
float prz = rz;

void setup()
{
  size(850, 480, P3D);   
  A1 = matIdentity(4); // create new 4x4 matrix
  A2 = matIdentity(4); // create new 4x4 matrix
  A3 = matIdentity(4); // create new 4x4 matrix
  A4 = matIdentity(4); // create new 4x4 matrix
  A5 = matIdentity(4); // create new 4x4 matrix
  A6 = matIdentity(4); // create new 4x4 matrix
  A7 = matIdentity(4); // create new 4x4 matrix
}

void draw()
{ 
  //println("draw1");
  background(127);
  
  stroke(255);
  strokeWeight(0.1);
    
  if(mousePressed)
  {
    
  rx += (pmouseX - mouseX) * PI / width;
  ry += (pmouseY - mouseY) * PI / height;
  //rz += sliderRZ.getValue()/(PI*1000);
  }
  else
  {
   hpx = -(mouseX - width/2) * 30.0 / width; 
   hpz = (height - mouseY) * 30.0 / height;
  //hpz -= sliderZ.getValue()/1000;
  
  }
  
  //println("draw2");
  if(calculateIK() == false)
  {
    hpx = phpx;
    hpy = phpy;
    hpz = phpz;
    
    rx = prx;
    ry = pry;
    rz = prz; 
    calculateIK();
  }
  else
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
  
  translate(-hpx,-hpy,hpz);
  fill(255,0,0);
  stroke(255,0,0);
  sphere(0.1);
  popMatrix();
  fill(255,255,255);
  
  pushMatrix();
  scale(10);
  stroke(0);
  fill(255);
  applyDHMatrix(A1);
  box(6,6,1);  
  applyDHMatrix(A2);
  
  pushMatrix();
  translate((a2)/2,0,0);
  box(a2-5,2,1);
  popMatrix();
  
  box(1,1,3);
  applyDHMatrix(A3);
  box(1,1,3);
  
  pushMatrix();
  translate(0,d4/2,0);
  box(2,d4-5,1);
  popMatrix();
  
  applyDHMatrix(A4);
  box(1,1,3);
  applyDHMatrix(A5);
  box(1,1,3);
  applyDHMatrix(A6);
//  box(1,1,3);

  pushMatrix();
  translate(0,0,tool/2);
  box(1,1,tool-5);
  popMatrix();
  
  applyDHMatrix(A7);
  fill(255,0,0);
  box(3,6,0.5);
  
  popMatrix();
  stroke(255);
  
}

boolean calculateIK()
{
  //println("calc1");
  
  float[][] rotation = rotationMatrix(rx,ry,rz);
  

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
   
    
  theta1 = atan2(py,px) - atan2(d3,sqrt(px*px + py*py - d3*d3));
  
  float K = (px*px + py*py + pz*pz - a2*a2 - a3*a3 - d3*d3 - d4*d4) / (2 * a2);
  
  theta3 = atan2(a3,d4) - atan2(K,-sqrt(a3*a3 + d4*d4 - K*K));
  
  float theta23 = atan2((-a3-a2*cos(theta3))*pz - (cos(theta1)*px + sin(theta1)*py)*(a2*sin(theta3) - d4),
                  (a2*sin(theta3)-d4)*pz - (a3+a2*cos(theta3))*(cos(theta1)*px + sin(theta1)*py));
  theta2 = theta23 - theta3;
  
  theta4 = atan2(-r13*sin(theta1)+r23*cos(theta1),-r13*cos(theta1)*cos(theta2+theta3) - r23*sin(theta1)*cos(theta2 + theta3) + r33*sin(theta2 + theta3));
  
  float s5 = -(r13*(cos(theta1)*cos(theta2 + theta3)*cos(theta4) +sin(theta1)*sin(theta4)) + r23*(sin(theta1)*cos(theta2+theta3)*cos(theta4) - cos(theta1)*sin(theta4)) - r33*(sin(theta2 + theta3)*cos(theta4)));
  float c5 = r13*(-cos(theta1)*sin(theta2 + theta3)) + r23*(-sin(theta1)*sin(theta2+theta3)) + r33*(-cos(theta2 + theta3));
  
  theta5 = atan2(s5,c5);
  
  float s6 = -r11*(cos(theta1)*cos(theta2 + theta3)*sin(theta4) - sin(theta1)*cos(theta4)) - r21*(sin(theta1)*cos(theta2 + theta3)*sin(theta4) + cos(theta1)*cos(theta4)) + r31*(sin(theta2 + theta3)*sin(theta4));
  float c6 = r11*((cos(theta1)*cos(theta2+theta3)*cos(theta4) + sin(theta1)*sin(theta4))*cos(theta5) - cos(theta1)*sin(theta2+theta3)*sin(theta5)) + r21*((sin(theta1)*cos(theta2+theta3)*cos(theta4) - cos(theta1)*sin(theta4))*cos(theta5) - sin(theta1)*sin(theta2+theta3)*sin(theta5))-r31*(sin(theta2+theta3)*cos(theta4)*cos(theta5) + cos(theta2+theta3)*sin(theta5));
  
  theta6 = atan2(s6,c6);
  
 // //println(px + "\t\t" + py + "\t\t" + pz + "\t\t" + atan2(d3,sqrt(px*px + py*py - d3*d3)));
  //println("calc2"); 
  A1 = DHMatrix(A1, 0,0,0,theta1);     // calculate initial DH Matrix
  A2 = DHMatrix(A2, 0,-PI/2,0,theta2);
  A3 = DHMatrix(A3, a2,0,0,theta3);
  A4 = DHMatrix(A4, 0,-PI/2,d4,theta4);
  A5 = DHMatrix(A5, 0,PI/2,0,theta5);
  A6 = DHMatrix(A6, 0,-PI/2,0,theta6);
  A7 = DHMatrix(A7, 0,0,tool,0);
  
  T1  = A1;                    // Calculate Positions
  T2 = matMultiply(T1,A2);
  T3 = matMultiply(T2,A3);
  T4 = matMultiply(T3,A4);
  T5 = matMultiply(T4,A5);
  T6 = matMultiply(T5,A6);
  T7 = matMultiply(T6,A7);
  //println("calc3");
  
  for(int i = 0; i < 3; i++)
  {
    //println("calc4");
    for(int j = 0; j < 3; j++)
    {
      //println("calc5");
      if(T7[i][j] == Float.NaN )
      {
         //println("calc6");
         return false;
      } 
      
      //println(T7[i][j]);
    } 
  }
  
  return true;
    
}

float[][] DHMatrix(float[][] Ai, float a_ , float alpha_ , float d_ , float theta_ )
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
  applyMatrix(Ai[0][0],Ai[0][1],Ai[0][2], Ai[0][3],
              Ai[1][0],Ai[1][1],Ai[1][2], Ai[1][3],
              Ai[2][0],Ai[2][1],Ai[2][2], Ai[2][3],
              Ai[3][0],Ai[3][1],Ai[3][2], Ai[3][3]);
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
    
  return matMultiply(Rx,matMultiply(Ry,Rz));
}



float[][] matMultiply(float a[][], float b[][]){//a[m][n], b[n][p]
   if(a.length == 0) return new float[0][0];
   if(a[0].length != b.length) return null; //invalid dims
 
   int n = a[0].length;
   int m = a.length;
   int p = b[0].length;
 
   float ans[][] = new float[m][p];
 
   for(int i = 0;i < m;i++){
      for(int j = 0;j < p;j++){
         for(int k = 0;k < n;k++){
            ans[i][j] += a[i][k] * b[k][j];
         }
      }
   }
   return ans;
}


float[][] matIdentity(int n)
{
   float[][] A = new float[n][n];
   for(int i = 0; i < n; i++)
     for(int j = 0; j < n; j++)
        if(i == j)
           A[i][j] = 1;
        else
           A[i][j] = 0;
   return A;
}

