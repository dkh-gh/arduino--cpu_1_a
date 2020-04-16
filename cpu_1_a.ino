/*************************************************
 * Прошивка для управления ЧПУ станком "Малютка" *
 * Версия программы 1.0                          *
 * Плата Arduino Uno                             *
 *************************************************/

int encoder[3] = {0, 0, 0};
int encoderPort[3] = {A0, A1, A2};
bool encoderLogic[3] = {true, true, true};
int encoderData[3] = {0, 0, 0};
int encoderDataMax[3] = {0, 0, 0};
int encoderDataMin[3] = {1023, 1023, 1023};
int encoderDataHigh[3] = {0, 0, 0};
int encoderDataLow[3] = {0, 0, 0};

float minX = 0;
float maxX = 80;
float minY = 0;
float maxY = 70;
float minZ = 0;
float maxZ = 5;

int motors = 3;
int motorPort[3] = {2, 5, 8};
bool motorWay[3] = {true, true, true};

bool debug = false;

void setup() {
  for( int i = 0; i < motors; i++ ) pinMode(encoderPort[i], INPUT);
  Serial.begin(115200);
  if(debug) Serial.println("setup()");
  encoderFirstCalibrate();
  Serial.println("ok");
}

String X="0", Y="0", Z="0";
char serialData[100];
int serialCount = 0;
void loop() {
  
  if(debug) Serial.println("loop()");
  
  if(Serial.available() > 0) {
    serialCount++;
    char s = Serial.read();
    serialData[serialCount] = s;
    //Serial.println(s);
    if(s == '\n') {
      //Serial.print("[");
      for( int i = 0; i < serialCount; i++ ) {
        if(serialData[i] == 'X') {
          X = "";
          i++;
          while(serialData[i] != ' ' && i < serialCount) {
            X += serialData[i];
            i++;
          }
        }
        if(serialData[i] == 'Y') {
          Y = "";
          i++;
          while(serialData[i] != ' ' && i < serialCount) {
            Y += serialData[i];
            i++;
          }
        }
        if(serialData[i] == 'Z') {
          Z = "";
          i++;
          while(serialData[i] != ' ' && i < serialCount) {
            Z += serialData[i];
            i++;
          }
        }
      }
      /*
      Serial.print("X:");
      Serial.print(X.toFloat());
      Serial.print(" Y:");
      Serial.print(Y.toFloat());
      Serial.print(" Z:");
      Serial.print(Z.toFloat());
      Serial.println("]");
      */
      float tmpZ = Z.toFloat();
      if(tmpZ < 0) tmpZ = tmpZ*(-1);
      moveTo(X.toFloat(), Y.toFloat(), tmpZ);
      serialCount = 0; // =================== ? ================ //
      Serial.println("ok");
    }
  }

  //Serial.println('G');
  //Serial.println(float(101.0/51.0));
  
}

void moveTo( float x, float y, float z ) {
  if(x > maxX) x = maxX;
  if(x < minX) x = minX;
  if(y > maxY) y = maxY;
  if(y < minY) y = minY;
  if(z > maxZ) z = maxZ;
  if(z < minZ) z = minZ;
  if(debug) Serial.println("moveTo()");
  int xyz[3] = {round(x/0.4), round(y/0.4), round(z/0.4)};
  
  for( int i = 0; i < motors; i++ ) {
    if( encoder[i] != xyz[i] ) {
      if( encoder[i] > xyz[i] ) {
        xyz[i] = encoder[i] - xyz[i];
        motorWay[i] = false;
      }
      else {
        xyz[i] = xyz[i] - encoder[i];
        motorWay[i] = true;
      }
    }
    else xyz[i] = 0;
  }
  
  while( xyz[0] > 0 || xyz[1] > 0 || xyz[2] > 0 ) {
    int oneStep[3];
    if(xyz[0] == xyz[1]) {
      oneStep[0] = 1;
      oneStep[1] = 1;
    }
    else {
      if(xyz[0] == 0 || xyz[1] == 0) {
        if(xyz[0] > 0) {
          oneStep[0] = 1;
          oneStep[1] = 0;
        }
        if(xyz[1] > 0) {
          oneStep[0] = 0;
          oneStep[1] = 1;
        }
      }
      else {
        if(xyz[0] > xyz[1]) {
          oneStep[0] = round(float(xyz[0])/float(xyz[1]));
          oneStep[1] = 1;
        }
        else {
          oneStep[0] = 1;
          oneStep[1] = round(float(xyz[1])/float(xyz[0]));
        }
      }
    }
    if(xyz[2] > 0) oneStep[2] = xyz[2];
    else oneStep[2] = xyz[2];
    
    if( xyz[2] > 0 ) for( int i = 0; i < oneStep[2]; i++ ) motorStep(2, motorWay[2]);
    if( xyz[0] > 0 ) for( int i = 0; i < oneStep[0]; i++ ) motorStep(0, motorWay[0]);
    if( xyz[1] > 0 ) for( int i = 0; i < oneStep[1]; i++ ) motorStep(1, motorWay[1]);
    
    /*
    Serial.print("#X ");
    Serial.print(xyz[0]);
    Serial.print(" Y ");
    Serial.print(xyz[1]);
    Serial.print(" X1 ");
    Serial.print(oneStep[0]);
    Serial.print(" Y1 ");
    Serial.println(oneStep[1]);
    */
    xyz[0] -= oneStep[0];
    xyz[1] -= oneStep[1];
    xyz[2] -= oneStep[2];
    
  }
}

void encoderCount() {
  if(debug) Serial.println("encoderCount()");
  encoderCalibrate();
  for( int i = 0; i < motors; i++ ) {
    if((encoderLogic[i] && encoderData[i] < encoderDataLow[i]) || (!encoderLogic[i] && encoderData[i] > encoderDataHigh[i])) {
      if(motorWay[i]) encoder[i]++;
      else encoder[i]--;
      encoderLogic[i] = !encoderLogic[i];
    }
  }
}

void encoderCalibrate() {
  if(debug) Serial.println("encoderCalibrate()");
  for( int i = 0; i < motors; i++ ) {
    encoderData[i] = analogRead(encoderPort[i]);
    if(encoderDataMax[i] < encoderData[i]) encoderDataMax[i] = encoderData[i];
    if(encoderDataMin[i] > encoderData[i]) encoderDataMin[i] = encoderData[i];
    int third = ((encoderDataMax[i] - encoderDataMin[i])/3);
    encoderDataHigh[i] = encoderDataMax[i] - third;
    encoderDataLow[i] = encoderDataMin[i] + third;
  }
  //serial("encoders");
}

void encoderFirstCalibrate() {
  if(debug) Serial.println("encoderFirstCalibrate()");
  int startTime = millis();
  for( int i = 0; i < motors; i++ ) motorRun(i, true, true);
  while(startTime+200 > millis()) encoderCalibrate();
  startTime = millis();
  for( int i = 0; i < motors; i++ ) motorRun(i, true, false);
  while(startTime+200 > millis()) encoderCalibrate();
  for( int i = 0; i < motors; i++ ) {
    motorRun(i, false, true);
    encoder[i] = 0;
  }
}

void motorStep( int num, bool way ) {
  if(debug) Serial.println("motorStep()");
  encoderCount();
  int startStep = encoder[num];
  motorRun(num, true, way);
  while( startStep == encoder[num] ) encoderCount();
  motorRun(num, false, way);
  serial("coordinate");
}

void motorRun( int num, bool rotate, bool way ) {
  if(debug) Serial.println("motorRun()");
  int motorSpeed = 100;
  if(rotate) {
    if(!way) motorSpeed *= -1;
    startMotor(motorPort[num], motorSpeed);
  }
  else stopMotorHard(motorPort[num]);
}

void endCode() {
  if(debug) Serial.println("endCode()");
  //serial("encoders");
  while(true) {}
}

void startMotor(int num, int spd) {
  digitalWrite(motorPort[num], 1);
  if(spd == 100) {
    digitalWrite(num+1, 1);
    digitalWrite(num+2, 0);
  }
  else if(spd == -100) {
    digitalWrite(num+1, 0);
    digitalWrite(num+2, 1);
  }
}

void stopMotorHard(int num) {
  digitalWrite(num, 1);
  digitalWrite(num+1, 1);
  digitalWrite(num+2, 1);
  delay(5);
  digitalWrite(num, 1);
  digitalWrite(num+1, 0);
  digitalWrite(num+2, 0);
  //delay(50);
}

void serial(String param) {
  if(debug) Serial.println("serial()");
  if(param == "coordinate") {
    Serial.print("X:");
    Serial.print(encoder[0]*0.4);
    Serial.print(" Y:");
    Serial.print(encoder[1]*0.4);
    Serial.print(" Z:");
    Serial.println(encoder[2]*0.4);
  }
  if(param == "encoders") {
    for( int i = 0; i < motors; i++ ) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(encoderDataMin[i]);
      Serial.print("/");
      Serial.print(encoderData[i]);
      Serial.print("/");
      Serial.print(encoderDataMax[i]);
      Serial.print("; ");
    }
    Serial.println();
  }
}
