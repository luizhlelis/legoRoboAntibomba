
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Adafruit_MotorShield.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

//Define valores usados pelos botões
int lcd_key     = 0;
int adc_key_in  = 0;
#define btnRIGHT  5
#define btnUP     4
#define btnDOWN   3
#define btnLEFT   2
#define btnSELECT 1
#define btnNONE   0

//Função para ler os botões
int read_LCD_buttons()
{
  adc_key_in = analogRead(0);      // read the value from the sensor 
  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
  // For V1.1 us this threshold
  if (adc_key_in < 50)   return btnRIGHT;  
  if (adc_key_in < 250)  return btnUP; 
  if (adc_key_in < 450)  return btnDOWN; 
  if (adc_key_in < 650)  return btnLEFT; 
  if (adc_key_in < 850)  return btnSELECT;  

  return btnNONE;  // when all others fail, return this...
}

//Pinos
#define leftEncoder 25
#define rigthEncoder 27
#define ldr A10 //LDR diferencial
const int ldrStart = A14; //LDR da partida
//Pinos conectados aos sensores óptico-reflexivos
const int rightSensor = A13;
const int leftSensor  = A8;
const int Bumper = 41;

//Variáveis Encoder
int leftEncState, rightEncState, oldLeftEncState, oldRightEncState, rightEncCount = 0, leftEncCount = 0;
double angRightMotor = 0, angLeftMotor = 0; // Angulo
double distRightMotor = 0, distLeftMotor = 0; // Distancia
int ang = 0, dist = 0; //Angulo em graus e distancia em cm.

//Variáveis Orientação
int ldrInputValue, minLdrValue, maxLdrValue, ligthLeftEncoderMax, ligthRightEncoderMax, ligthLeftEncoderMin, ligthRightEncoderMin;

//Variáveis Partida
int startValueLampOff, ldrStartValue;

//Variáveis Line-Following
//Valores de leitura dos sensores
int rightValue = 0;
int leftValue = 0;
//Variáveis para as superfícies
int whiteSurfaceR = 0;
int blackSurfaceR = 0;
int whiteSurfaceL = 0;
int blackSurfaceL = 0;
int setPointR = 0;
int setPointL = 0;
//Variaveis Line-Following PID
int Kp = 0;
int Ki = 0;
int Kd = 0;
int correctionR = 0;
int correctionL = 0;
int errorR = 0;
int errorL = 0;
int lastErrorR = 0;
int lastErrorL = 0;
int integralR = 0;
int integralL = 0;
int derivativeR = 0;
int derivativeL = 0;

//Variável Bumper
int stateBumper, offBumper;

//Intervalo de mudança do Display
const long intervalo_display = 400;
//Intervalo de parada
const long intervalo_parada = 60000;
//Variáveis para armazenar tempo
unsigned long tempo_anterior = 0, tempo_atual = 0, tempo_inicial;

//Criando o objeto motor shield com o endereco default I2C
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Selecionando a porta de cada motor
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rigthMotor = AFMS.getMotor(4);

//Funções
void ldrStartCalibrate();
void LightSensorsCalibrate();
void Orientation();
void moveForward();
void LineFollowing();
void moveCurve();


void setup()  { 

  //Display com 16 colunas e 2 linhas
  lcd.begin(16, 2);
  
  //Inicia o objeto de motor com a frequencia default = 1.6KHz
  AFMS.begin();
  
  Serial.begin(9600);
  Serial.print("leftEncCount = " );                       
  Serial.println(leftEncCount);
  Serial.print("rightEncCount = " );                       
  Serial.println(rightEncCount);

  pinMode(leftEncoder,INPUT);
  pinMode(rigthEncoder,INPUT);

  pinMode(Bumper,INPUT);
  
} 

void loop() {  

  int i, j, tamanho;
  
  //Tela Inicial
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Mandela Cabuloso");
  lcd.setCursor(0,1);
  lcd.print("Iniciar (SELECT)");           
  while(lcd_key != 1){
    lcd_key = read_LCD_buttons();  //Lê os botões
  }
  delay(300);
  lcd_key = 0;

//Selecionar tarefas
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Escolha a tarefa");
  char texto1[] = "1-Calibracao(LEFT) 2-Competicao(DOWN)"; //37 caracteres
  tamanho = 37;
  for(i = 0; i < 16; i++){
    lcd.setCursor(i,1);
    lcd.print(texto1[i]);
  }
  
  i = 16;
  tempo_anterior = millis();
  while((lcd_key != 2)&&(lcd_key != 3)){    
    lcd_key = read_LCD_buttons();
    tempo_atual = millis();
    if(tempo_atual - tempo_anterior >= intervalo_display){
      tempo_anterior = tempo_atual;
      if(i < tamanho){
        for(j = 0; j < 16; j++){
          lcd.setCursor(j,1);
          lcd.print(texto1[i-15+j]);
        }
        i++;
      }
      else{
        for(j = 0; j < 16; j++){
          lcd.setCursor(j,1);
          lcd.print(texto1[j]);
        }
        i = 16;
      }
    }
  }

//Calibracao
  if(lcd_key == 2){
    delay(300);
    lcd_key = 0;
    
    //Line-Following
    LightSensorsCalibrate();
    
    //LDR Partida
    ldrStartCalibrate();
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("CALIBRACAO");
    lcd.setCursor(0,1);
    lcd.print("PRONTA!");
    delay(500);
  }

//Competição
  if(lcd_key == 3){
    delay(300);
    lcd_key = 0;

    offBumper = 0;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LDR Partida:");

    tempo_anterior = millis();
    while(1){
      ldrStartValue = analogRead(ldrStart);
      tempo_atual = millis();
      if(tempo_atual - tempo_anterior >= intervalo_display){
        tempo_anterior = tempo_atual;
        lcd.setCursor(0,1);
        lcd.print(ldrStartValue);
        lcd.print(" ");
      }
      if(ldrStartValue >= 900){
        tempo_inicial = millis();
        break;
      }
    }
   
    lcd.setCursor(0,1);
    lcd.print(ldrStartValue);

    Orientation();
    
    dist = 40;
    moveForward();

    Kp = 1;
    Ki = 0;
    Kd = 0;

    setPointR = (blackSurfaceR - whiteSurfaceR)/2 + whiteSurfaceR;
    setPointL = (blackSurfaceL - whiteSurfaceL)/2 + whiteSurfaceL;

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("LDR Partida:");

    tempo_anterior = millis();
    while(1){
      ldrStartValue = analogRead(ldrStart);
      tempo_atual = millis();
      if(tempo_atual - tempo_inicial >= intervalo_parada){
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("60 Segundos!");
        delay(30000);
      }
      if(tempo_atual - tempo_anterior >= intervalo_display){
        tempo_anterior = tempo_atual;
        lcd.setCursor(0,1);
        lcd.print(ldrStartValue);
        lcd.print(" ");
      }
      //Detectou bloco
      if(ldrStartValue <= 70){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Blc Identificado");
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        break;
      }
      
      LineFollowing();
    }

    dist = 5;
    moveForward();

    ang = 180;
    moveCurve();

    while(1){
      tempo_atual = millis();
      if(tempo_atual - tempo_inicial >= intervalo_parada){
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("60 Segundos!");
        delay(10000);
        break;
      }
      stateBumper = digitalRead(Bumper);
      if((stateBumper == HIGH)&&(offBumper != 1)){
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Bumper!");
        offBumper = 1;
      }
      if(offBumper != 1){
        LineFollowing();
      }
    }
  }
}
//Fim loop()

//Movimentação com giro
void moveCurve(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Giro");

  leftEncCount = 0;
  rightEncCount = 0;

  angRightMotor = ang*0.070;
  angLeftMotor = ang*0.070;

  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(150);
  rigthMotor->run(FORWARD);
  rigthMotor->setSpeed(145);

  while(1){
    //Leituras
    leftEncState = digitalRead(leftEncoder);
    rightEncState = digitalRead(rigthEncoder);
    // Verifica borda de descida do encoder esquerdo
    if (leftEncState == HIGH){
      if (oldLeftEncState == LOW){
        leftEncCount++;
      }
    } else{ //Verifica borda de subida
      if (oldLeftEncState == HIGH){
        leftEncCount++;
      }
    }
  
    // Verifica borda de descida do encoder direito
    if (rightEncState == HIGH){
      if (oldRightEncState == LOW){
        rightEncCount++;
      }
    } else { //Verifica borda de subida
      if (oldRightEncState == HIGH){
        rightEncCount++;
      }
    }
    
    oldRightEncState = rightEncState;
    oldLeftEncState = leftEncState;

    if((rightEncCount >= angRightMotor)&&(leftEncCount >= angLeftMotor)){
      leftMotor->run(RELEASE);
      rigthMotor->run(RELEASE);
      break;
    }
  }
}

//Line-Following
void LineFollowing(){
  /*lcd.setCursor(5,0);
  lcd.print(Kp);
  lcd.setCursor(5,1);
  lcd.print(Ki);
  /*lcd.setCursor(5,1);
  lcd.print(Kd);*/
    
    //Leitura dos sensores
    rightValue = analogRead(rightSensor);
    leftValue = analogRead(leftSensor);

    //Erro = Setpoint - leitura
    errorR = setPointR - rightValue;
    errorL = setPointL - leftValue;
    integralR += errorR;
    integralL += errorL;
    derivativeR = errorR - lastErrorR;
    derivativeL = errorL - lastErrorL;
    correctionR = (Kp * errorR) + (Ki * integralR) + (Kd * derivativeR);
    correctionL = (Kp * errorL) + (Ki * integralL) + (Kd * derivativeL);

    //Move para frente
    if(rightValue < setPointR && leftValue < setPointL){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(150);
      rigthMotor->run(FORWARD);
      rigthMotor->setSpeed(145);
    }
    //Move para direita
    else if(rightValue > setPointR && leftValue < setPointL){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(150);
      rigthMotor->run(RELEASE);
    }
    //Move para esquerda
    else if(rightValue < setPointR && leftValue > setPointL){
      rigthMotor->run(FORWARD);
      rigthMotor->setSpeed(150);
      leftMotor->run(RELEASE);
    }
    //Segue em frente também, caso ambos sobre a linha preta
    else if(rightValue > setPointR && leftValue > setPointL){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(150);
      rigthMotor->run(FORWARD);
      rigthMotor->setSpeed(145);
    }

    //Up e Down alteraram Kp durante execução
    lcd_key = read_LCD_buttons();
    if(lcd_key == 4){
      delay(300);
      lcd_key = 0;
      Kp += 1;
    }
    if(lcd_key == 3){
      delay(300);
      lcd_key = 0;
      Kp -= 1;
    }

    //Left e Right alteraram Ki durante execução
    lcd_key = read_LCD_buttons();
    if(lcd_key == 2){
      delay(300);
      lcd_key = 0;
      Ki -= 1;
    }
    if(lcd_key == 5){
      delay(300);
      lcd_key = 0;
      Ki += 1;
    }
}

//Função move para frente
void moveForward(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Frente");

  leftEncCount = 0;
  rightEncCount = 0;

  distRightMotor = dist*0.46; 
  distLeftMotor = dist*0.46;

  leftMotor->run(FORWARD);
  leftMotor->setSpeed(150);
  rigthMotor->run(FORWARD);
  rigthMotor->setSpeed(145);

  while(1){
      leftEncState = digitalRead(leftEncoder);
      rightEncState = digitalRead(rigthEncoder);
      
      // Verifica borda de descida do encoder esquerdo
      if (leftEncState == HIGH){
        if (oldLeftEncState == LOW){
          leftEncCount++;
        }
      } else{ //Verifica borda de subida
        if (oldLeftEncState == HIGH){
          leftEncCount++;
        }
      }
    
      // Verifica borda de descida do encoder direito
      if (rightEncState == HIGH){
        if (oldRightEncState == LOW){
          rightEncCount++;
        }
      } else { //Verifica borda de subida
        if (oldRightEncState == HIGH){
          rightEncCount++;
        }
      }
      
      oldRightEncState = rightEncState;
      oldLeftEncState = leftEncState;
    
      if((rightEncCount >= distRightMotor)&&(leftEncCount >= distLeftMotor)){
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        break;
      }
  }
}

//Função pra orientação segundo luzes polarizadas
void Orientation(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Orientacao");

  // Atribuindo os valores iniciais
  leftEncCount = 0;
  rightEncCount = 0;
  ldrInputValue = 9999; // Variavel para armazenar a leitura atual do sensor ldr
  minLdrValue = 9999;
  maxLdrValue = 0;
  ligthLeftEncoderMax  = 0; // Pontos do encoder que ocorreram a iluminacao maxima
  ligthRightEncoderMax = 0;
  ligthLeftEncoderMin  = 0; // Pontos do encoder que ocorreram a iluminacao maxima
  ligthRightEncoderMin = 0;

  angRightMotor = 360*0.070;
  angLeftMotor = 360*0.070;

  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(150);
  rigthMotor->run(FORWARD);
  rigthMotor->setSpeed(145);

  //Procura luz
  while(1){
    //Leituras
    leftEncState = digitalRead(leftEncoder);
    rightEncState = digitalRead(rigthEncoder);
    // Verifica borda de descida do encoder esquerdo
    if (leftEncState == HIGH){
      if (oldLeftEncState == LOW){
        leftEncCount++;
      }
    } else{ //Verifica borda de subida
      if (oldLeftEncState == HIGH){
        leftEncCount++;
      }
    }
  
    // Verifica borda de descida do encoder direito
    if (rightEncState == HIGH){
      if (oldRightEncState == LOW){
        rightEncCount++;
      }
    } else { //Verifica borda de subida
      if (oldRightEncState == HIGH){
        rightEncCount++;
      }
    }

    // Faz a leitura do sensor ldr
     ldrInputValue = analogRead(ldr);

     // Verifica ponto menos e mais luminoso
     if(ldrInputValue < minLdrValue) {
        minLdrValue = ldrInputValue;
        ligthLeftEncoderMin  = leftEncCount;
        ligthRightEncoderMin = rightEncCount;
     }
     else if(ldrInputValue > maxLdrValue) {
        maxLdrValue = ldrInputValue;
        ligthLeftEncoderMax  = leftEncCount;
        ligthRightEncoderMax = rightEncCount;
     }
    
    oldRightEncState = rightEncState;
    oldLeftEncState = leftEncState;

    if((rightEncCount >= angRightMotor)&&(leftEncCount >= angLeftMotor)){
      leftMotor->run(RELEASE);
      rigthMotor->run(RELEASE);
      break;
    }
  }

  leftEncCount = 0;
  rightEncCount = 0;
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(150);
  rigthMotor->run(FORWARD);
  rigthMotor->setSpeed(145);
  //Volta para posição da luz
  while(1){
    //Leituras
    leftEncState = digitalRead(leftEncoder);
    rightEncState = digitalRead(rigthEncoder);
    // Verifica borda de descida do encoder esquerdo
    if (leftEncState == HIGH){
      if (oldLeftEncState == LOW){
        leftEncCount++;
      }
    } else{ //Verifica borda de subida
      if (oldLeftEncState == HIGH){
        leftEncCount++;
      }
    }
  
    // Verifica borda de descida do encoder direito
    if (rightEncState == HIGH){
      if (oldRightEncState == LOW){
        rightEncCount++;
      }
    } else { //Verifica borda de subida
      if (oldRightEncState == HIGH){
        rightEncCount++;
      }
    }

    oldRightEncState = rightEncState;
    oldLeftEncState = leftEncState;

    if(minLdrValue > 450) {
      if((leftEncCount >= ligthLeftEncoderMax) && (rightEncCount >= ligthRightEncoderMax)){
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        break;
      }
    }
    else {
      if((leftEncCount >= ligthLeftEncoderMin) && (rightEncCount >= ligthRightEncoderMin)){
        leftMotor->run(RELEASE);
        rigthMotor->run(RELEASE);
        break;
      }
    }
  }

}

//Calibração dos sensores óptico-reflexivos
void LightSensorsCalibrate(){
  int i;
  //Direita - Branco
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sensor Direita");
  lcd.setCursor(0,1);
  lcd.print("Branco:         ");
  while(lcd_key != 1){
    lcd_key = read_LCD_buttons();
  }
  if(lcd_key == 1){
    lcd_key = 0;
    rightValue = 0;
    for(i = 0; i < 10; i++){
      rightValue += analogRead(rightSensor);
    }
    whiteSurfaceR = rightValue/10;
    lcd.setCursor(8,1);
    lcd.print(whiteSurfaceR);
  }
  delay(500);

  //Esquerda - Branco
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sensor Esquerda");
  lcd.setCursor(0,1);
  lcd.print("Branco:         ");
  while(lcd_key != 1){
    lcd_key = read_LCD_buttons();
  }
  if(lcd_key == 1){
    lcd_key = 0;
    leftValue = 0;
    for(i = 0; i < 10; i++){
      leftValue += analogRead(leftSensor);
    }
    whiteSurfaceL = leftValue/10;
    lcd.setCursor(8,1);
    lcd.print(whiteSurfaceL);
  }
  delay(500);

  //Direita - Preto
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sensor Direita");
  lcd.setCursor(0,1);
  lcd.print("Preto:          ");
  while(lcd_key != 1){
    lcd_key = read_LCD_buttons();
  }
  if(lcd_key == 1){
    lcd_key = 0;
    rightValue = 0;
    for(i = 0; i < 10; i++){
      rightValue += analogRead(rightSensor);
    }
    blackSurfaceR = rightValue/10;
    lcd.setCursor(8,1);
    lcd.print(blackSurfaceR);
  }
  delay(500);

  //Esquerda - Preto
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sensor Esquerda");
  lcd.setCursor(0,1);
  lcd.print("Preto:          ");
  while(lcd_key != 1){
    lcd_key = read_LCD_buttons();
  }
  if(lcd_key == 1){
    lcd_key = 0;
    leftValue = 0;
    for(i = 0; i < 10; i++){
      leftValue += analogRead(leftSensor);
    }
    blackSurfaceL = leftValue/10;
    lcd.setCursor(8,1);
    lcd.print(blackSurfaceL);
  }
  delay(500);
}

//Calibração LDR de Partida
void ldrStartCalibrate(){
  int i;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor Partida  ");
  lcd.setCursor(0, 1);
  lcd.print("Lamp Des:       ");
  while(lcd_key != 1){
    lcd_key = read_LCD_buttons();
  }
  if(lcd_key == 1){
    lcd_key = 0;
    startValueLampOff = 0;
    for(i = 0; i < 10; i++){
      startValueLampOff += analogRead(ldrStart);
    }
    startValueLampOff /= 10;
    lcd.setCursor(10, 1);
    lcd.print(startValueLampOff);
  }
  delay(500);
}

