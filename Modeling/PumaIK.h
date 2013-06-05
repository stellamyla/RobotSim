#ifndef PUMA_IK_H
#define PUMA_IK_H

//given denavit-hartenberg parameters for links 3 and 4
//given position, euler angles of desired toolpoint position
//produces 4 configurations
inline void PumaIK(Real a3,Real a4,Real d3,Real d4,Real Px,Real Py,Real Pz,Real theta4,Real theta5,Real theta6,vector<Vector>& configs);

inline void Puma760IK(Real Px,Real Py,Real Pz,Real theta4,Real theta5,Real theta6,vector<Vector>& configs)
{
  PumaIK(0.650,0.0,0.190,0.600,Px,Py,Pz,theta4,theta5,theta6,configs);
}

#endif

inline void PumaIK(Real a3,Real a4,Real d3,Real d4,Real Px,Real Py,Real Pz,Real theta4,Real theta5,Real theta6,vector<Vector>& configs)
{
  configs.resize(0);
  Vector config(6,Zero);

  //Real rho = Sqrt(Sqr(Px)+Sqr(Py));
  //Real phi = Atan2(Py,Px);
  Real K = (Sqr(Px)+Sqr(Py)+Sqr(Pz)-Sqr(a3)-Sqr(a4)-Sqr(d3)-Sqr(d4))/(2*a3);
  Real c1,c2,c3,c4,c5,c5,s1,s2,s3,s4,s5,s6;
  Real t23,s23,c23,r13,r23,r33,r11,r21,r31;
  c4 = Cos(theta4);
  s4 = Sin(theta4);
  c5 = Cos(theta5);
  s5 = Sin(theta5);
  c6 = Cos(theta6);
  s6 = Sin(theta6);

  Real sign1=1,sign2=1;
  for (int i=1;i<=4;i++) {
    if (i == 1) {
      sign1 = 1;
      sign3 = 1;
    }
    else if (i == 2) {
      sign1 = 1;
      sign3 = -1;
    }
    else if(i == 3) {
      sign1 = -1;
      sign3 = 1;
    }
    else {
      sign1 = -1;
      sign3 = -1;
    }
    config(0) = (Atan2(Py,Px)-Atan2(d3,sign1*Sqrt(Sqr(Px)+Sqr(Py)-Sqr(d3))));
    
    c1 = Cos(config(0));
    s1 = Sin(config(0));
    config(2) = (Atan2(a3,d4)-Atan2(K,sign3*Sqrt(Sqr(a4)+Sqr(d4)-Sqr(K))));
    
    c3 = Cos(config(2));
    s3 = Sin(config(2));
    t23 = Atan2((-a4-a3*c3)*Pz-(c1*Px+s1*Py)*(d4-a3*s3),(a3*s3-d4)*Pz+(a4+a3*c3)*(c1*Px+s1*Py));
    config(1) = (t23 - config(2));
    
    c2 = Cos(config(1));
    s2 = Sin(config(1));
    s23 = ((-a4-a3*c3)*Pz+(c1*Px+s1*Py)*(a3*s3-d4))/(Sqr(Pz)+Sqr(c1*Px+s1*Py));
    c23 = ((a3*s3-d4)*Pz+(a4+a3*c3)*(c1*Px+s1*Py))/(Sqr(Pz)+Sqr(c1*Px+s1*Py));
    r13 = -c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
    r23 = -s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
    r33 = s23*c4*s5 - c23*c5;
    config(3) = Atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);

    r11 = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)+s1*(s4*c5*c6+c4*s6);
    r21 = s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6);
    r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;
    s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
    c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
    config(4) = Atan2(s5,c5);

    s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
    c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
    config(5) = Atan2(s6,c6);
    
    //TODO: there may be something weird with config(1)
    configs.push_back(config);
  }
}
