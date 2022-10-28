
#include <LowPower.h>           //Incluye la libreria de LowPower
#include <Simple_MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#define MPU6050_ADDRESS_AD0_LOW   0x68
#define OFFSETS   2462,     602,     690,     176,      34,     -24  //Valores personalizados
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis()-SpamTimer) >= (t); SpamTimer = millis()) //función para mostrar datos en el monitor serie
#define printfloatx(Name, Variable,Spaces,Precision,EndTxt) print(Name);{char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision,S));}Serial.print(EndTxt);
TinyGPS gps;                     //Declaracion de objeto gps
SoftwareSerial serialgps(4, 3);  //Declaración de pin 4 Tx y 3 Rx

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();

unsigned long delayTime;        //Tipo de variable para delay
int aux = 0;                    //Variable de contador
float flat, flon;               //Varieble de latitud y longitud
//*****************************************************
String bufer; //variable donde guardaremos nuestro payload
String bufer2="\n";   //agregamos un salto de linea al final de nuestro payload
//*****************************************************

void setup() {
 Serial.begin(9600);           //Velocidad del puerto
// Serial.begin(115200);       //Velocidad del puerto
 pinMode(7, OUTPUT);           //enable modulo wisol
   pinMode(5, OUTPUT);           //enable modulo wisol
 serialgps.begin(9600);

  uint8_t val;
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == 12CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400,true);
  #endif

//  while(!Serial);
  Serial.print(F("Inicio:"));
  #ifdef OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);
  #else
  Serial.println(" No se establecieron offsets, haremos unos nuevos.\n"
                 " Colocar el sensor en una superficie plana y esperar unos segundos\n"
                 " Colocar los nuevos Offsets en #define OFFSETS\n"
                 " para saltar la calibracion inicial \n"
                 " \t\tPresiona cualquier tecla y ENTER");               
  while(Serial.available() && Serial.read());
  while(!Serial.available());
  while(Serial.available() && Serial.read());
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();
  #endif
  mpu.on_FIFO(mostrar_valores);
}

void loop() 
{
    digitalWrite(5,HIGH);
    mpu.dmp_read_fifo();
}


///////////////////////////////////Funcion add_float/////////////////////
void add_float(float var1) 
{
  byte* a1 = (byte*) &var1;    //convertimos el dato a bytes
  String str1;
  //agregamos al comando AT$SF= nuestra informacion a enviar
  for(int i=0;i<4;i++)
  {
    str1=String(a1[i], HEX);    //convertimos el valor hex a string 
    if(str1.length()<2)
    {
      bufer+=0+str1;    //si no, se agrega un cero
    }
    else
    {
      bufer+=str1;    //si esta completo, se copia tal cual
    }
  }
}
///////////////////////////////////Funcion add_int/////////////////////
void add_int(int var2)    //funcion para agregar enteros al payload (hasta 255)
{
  byte* a2 = (byte*) &var2; //convertimos el dato a bytes
  String str2;
  str2=String(a2[0], HEX);  //convertimos el valor hex a string 
  //verificamos si nuestro byte esta completo
  if(str2.length()<2)
  {
    bufer+=0+str2;    //si no, se agrega un cero
  }
  else
  {
    bufer+=str2;     //si esta completo, se copia tal cual
  }
}

void send_message()
{
  //agregamos el salto de linea "\n"
  bufer+=bufer2;
  //*******************
  //Habilitamos el modulo Sigfox
  digitalWrite(7, HIGH);
  delay(2000);
  //Reset del canal para asegurar que manda en la frecuencia correcta
  Serial.print("AT$RC\n"); 
  delay(2000);
  //************************
  //Enviamos la informacion por sigfox
  Serial.print(bufer);
  delay(6000);
  //deshabilitamos el modulo Sigfox
  digitalWrite(7, LOW);
  delay(500);
}
void mostrar_valores (int16_t *gyro, int16_t*accel,int32_t *quat, uint32_t *timestamp)
{
  uint8_t SpamDelay = 100;  //Demora para mostrar los valores en el monitor serie
  Quaternion q;             //Variable de la libreria para obtener información del sensor
  VectorFloat gravity;      //Variable de la libreria para obtener información del sensor
  float ypr[3]={0,0,0};     //Posiciones Yam,Pitch y Roll
  float xyz[3]={0,0,0};     //Ejes
  spamtimer(SpamDelay)
  {
    mpu.GetQuaternion(&q,quat);            //Funciones de la libreria para hacer calculo de posicion Yam,Pitch y Roll
    mpu.GetGravity(&gravity,&q);           //Funciones de la libreria para hacer calculo de posicion Yam,Pitch y Roll
    mpu.GetYawPitchRoll(ypr,&q,&gravity);  //Función para almacenar los valores Yam,Pitch y Roll obtenidos
    mpu.ConvertToDegrees(ypr,xyz);         //Convierte los valores a grados hexadecimales
    Serial.printfloatx(F("Yaw")   , xyz[0],9,4,F(",  "));  //eje z
    Serial.printfloatx(F("Pitch") , xyz[1],9,4,F(",  "));  //eje y
    Serial.printfloatx(F("Roll")  , xyz[2],9,4,F(",  "));  //eje x
    Serial.println();
    if( xyz[1] > 74 | xyz[1] < -74)
    {
      Serial.println("ALERTA");
      read_coordenadas();
      send_message(); 
      delay(10000);
    }  
  }
}

void read_coordenadas()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (serialgps.available())
    {
      char c = serialgps.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    bufer="AT$SF=";  //Comando para mandar por sigfox maximo 12 bytes
    add_float(flat);//Latitud
    add_float(flon);//Longitud
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  }
  gps.stats(&chars, &sentences, &failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **"); 
}
