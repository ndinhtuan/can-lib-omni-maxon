// thêm thư viện CAN vào đây, sau đó sửa code từ dòng 73 - 106

//
#include <math.h>
#include <stdio.h>
// #include <LibCan.h>
#define MOT_OFFSET  10 //chỉ số lệch sau khi tính toán, dòng 175-180
#define LIMIT_OFF 0 // tham số hạn biên giá trị xuất ra động cơ có/không 
#define LIMIT_ON 1

float fconstrain(float x, float a, float b); // hàm biên giá trị, x vào, a là biên -, b là biên +
int map(float data, float sLE, float sRE, float tLE, float tRE);// maping từ % sang tốc độ thực 
class RPomniDirect // class xuất lệnh điều khiển 
{
  public:
  RPomniDirect(float a_gain, float b_gain , float c_gain, float f_gain);  
  RPomniDirect(float a_gain, float b_gain , float c_gain, float f_gain, int limit);
  RPomniDirect(float a_gain, float b_gain , float c_gain, float f_gain,int motc,int motb,int mota);

  void move(int vx, int vy, int w, int *speed0, int *speed1, int *speed2);
  void motor(float speed, int ch);
  private:
  float _a_gain;
  float _b_gain;
  float _c_gain;
  float _f_gain;
  int _limit;
  int _mota;
  int _motb;
  int _motc;
    };

const int _mot[4]={2,1,0,3};

RPomniDirect::RPomniDirect(float a_gain, float b_gain , float c_gain, float f_gain)
{
  _a_gain = a_gain;
  _b_gain = b_gain;
  _c_gain = c_gain;
  _f_gain = f_gain;
  _limit = 1;
  _mota=2;
  _motb=1;
  _motc=0;
}

// add 2015/10/6
RPomniDirect::RPomniDirect(float a_gain, float b_gain , float c_gain, float f_gain,int motc,int motb,int mota)
{
  _a_gain = a_gain;
  _b_gain = b_gain;
  _c_gain = c_gain;
  _f_gain = f_gain;
  _mota=mota;
  _motb=motb;
  _motc=motc;
  _limit = 1;
}

RPomniDirect::RPomniDirect(float a_gain, float b_gain , float c_gain, float f_gain, int limit)
{
  _a_gain = a_gain;
  _b_gain = b_gain;
  _c_gain = c_gain;
  _f_gain = f_gain;
  _limit = limit;
  _mota=2;
  _motb=1;
  _motc=0;
}

void RPomniDirect::motor(float speed, int ch) // khi nào có thư viện CAN thì thay hàm vào đây để ra lệnh
{ 
 int speed_conv =  map(fconstrain(speed,-100,100),-100,100,-2000,2000); 
  switch (ch)
  {
  case 0:
    //_AMOT.rotate(map(constrain(speed,-100,100),-100,100,-2000,2000));
    //canlib(speed);
    //Serial.println(map(constrain(speed,-100,100),-100,100,-2000,2000));
    //printf("0:%d \n",  map(fconstrain(speed,-100,100),-100,100,-2000,2000)
    // printf("0:%d ", speed);
    
    break;
  case 1:
    //_BMOT.rotate(map(constrain(speed,-100,100),-100,100,-2000,2000));
    //canlib(speed);
    //Serial.println(map(constrain(speed,-100,100),-100,100,-2000,2000));
    //printf("1:%d \n", map(fconstrain(speed,-100,100),-100,100,-2000,2000));
    // printf("1:%d ", speed);
    break;
  case 2:
    //_CMOT.rotate(map(constrain(speed,-100,100),-100,100,-2000,2000));
    //canlib(speed);
    //Serial.println(map(constrain(speed,-100,100),-100,100,-2000,2000));
    //printf("2:%d\n", map(fconstrain(speed,-100,100),-100,100,-2000,2000));
    // printf("2:%d ", speed);
    break;
   case 3:
    //_DMOT.rotate(map(constrain(speed,-100,100),-100,100,-2000,2000);                  0));
    //canlib(speed);
    //Serial.println(map(constrain(speed,-100,100),-100,100,-2000,2000));
    printf("3:%d ", map(fconstrain(speed,-100,100),-100,100,-2000,2000));
    // printf("3:%d ", speed);
    break;
  }
}

float fconstrain(float x, float a, float b)
{
  if(x<=a)return(a);  
  if(x>=b)return(b);  
  return(x);
}

int map(float data, float sLE, float sRE, float tLE, float tRE)
{
	int d1 = tRE / (float(sRE/data));
	return d1;
}

//-------------------------------
void RPomniDirect::move(int vx, int vy, int  w, int *speed0, int *speed1, int *speed2)
{ 
  float a = 0;
  float b = 0;
  float c = 0;

  // adjustment w
  /*
if(sqrt(vx*vx+vy*vy)<=10&&vx>0) w += 50*pow(cos(3*atan2(vy,vx)),2)*sqrt(vx*vx+vy*vy)/10;
   if(sqrt(vx*vx+vy*vy)<=10&&vx<0) w -= 50*pow(cos(3*atan2(vy,vx)),2)*sqrt(vx*vx+vy*vy)/10;
   */
  
  if(_limit)
  {
    /*リミッター*/
    vx=fconstrain(vx,-100,100);
    vy=fconstrain(vy,-100,100);
    w=fconstrain(w,-100,100);
  }
  
    w=-6*w;

  /*`摩擦補正 */
  if(sqrt(vx*vx+vy*vy)<=10)
  {
    vx=0;
    vy=0;
  }
  if(sqrt(vx*vx+vy*vy)>10&&vx>0) w -= _f_gain*pow(cos(3*atan2(vy,vx)),2);
  if(sqrt(vx*vx+vy*vy)>10&&vx<0) w += _f_gain*pow(cos(3*atan2(vy,vx)),2);

  a -=   (float)vx;
  a += 0.1f * (float)w;

  b = 0.5f * (float)vx;
  b -= 0.866f * (float)vy;
  b += 0.1f * (float)w;

  c = 0.5f * (float)vx;
  c += 0.866f * (float)vy;
  c += 0.1f * (float)w;

  a=_a_gain*a;
  b=_b_gain*b;
  c=_c_gain*c;

  if(a > 0.0f) a += MOT_OFFSET;
  else if(a < 0.0f) a -= MOT_OFFSET;
  if(b > 0.0f) b += MOT_OFFSET;
  else if(b < 0.0f) b -= MOT_OFFSET;
  if(c > 0.0f) c += MOT_OFFSET;
  else if(c < 0.0f) c -= MOT_OFFSET;
/*
  motor((int)-a ,2); //amot
  motor((int)-b ,1); //bmot
  motor((int)-c ,0); //cmot
  */
  
  //return speed to speed0, speed1, speed2
  *speed0 = map(fconstrain(-a,-100,100),-100,100,-2000,2000);
  *speed1 = map(fconstrain(-b,-100,100),-100,100,-2000,2000);
  *speed2 = map(fconstrain(-c,-100,100),-100,100,-2000,2000);
  
  //motor((int)-a ,_mot[_motc]); //amot 0
  //motor((int)-b ,_mot[_motb]); //bmot 1
  //motor((int)-c ,_mot[_mota]); //cmot 2
}


// int main(int argc, char **argv) 
// {
  // RPomniDirect rb(1.0f,1.0f,1.0f,50.0f); // tạo 1 đối tượng là rb có thuộc tính của class RPomniDirect
  // int speed0, speed1, speed2;
  // speed0 = 0;
  // speed1 = 0;
  // speed2 = 0;
  // rb.move(0, 50, 0, &speed0, &speed1, &speed2); // xuất lệnh điều khiển robot đi thẳng với 50% tốc độ max
  // printf("%d %d %d", speed0, speed1, speed2);
  // return 0;
// }
