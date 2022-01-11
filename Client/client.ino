#include <ESP32Servo.h>                 //Necessário para controlar servomotor.
#include <WiFi.h>                       //Necessário para conexão com rede wifi.
#include <NTPClient.h>                  //Necessário para obter o horário de Brasília.
#include <WiFiUdp.h>                    //Biblioteca UDP para NTPClient conseguir pegar informações. Também será utilizada para trocar de informações entre os esp32.
#include <FS.h>                         //Biblioteca de FileSystem. Permite que haja módulos de comunicação e gestão de arquivos.
#include "SD.h"                         //Biblioteca para utilização do módulo cartão sd.
#include <SPI.h>                        //Biblioteca Serial. Utilizada para comunicação com outros dispositivos. Utilizam as portas MISO/MOSI/SCK
#include <Wire.h>                       //Necessário para INA219 (I2C/SDA//SCL)
#include <Adafruit_INA219.h>            //Necessário para INA219
#include "math.h"

/* Definições trigonométricas */
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

bool ativarBloqueioPeloHorario = false; //Serve para habilitar o bloqueio do funcionamento do rastreador pela hora atual. Por exemplo, se o valor for true, o rastreador só irá funcionar nos horários entre 07AM e 07PM.

/* Variáveis Gerais */
int ledPin = 19;                         //Pino do led que demostra o status da comunicação
int loopTime = 60000;                   //Tempo para realizar o loop em ms.
double tolerancia_Voltagem = 1.3;       //Valor aceitável de aumento, em porcentagem, da tensão anterior para a atual. Se for satisfatório, o client enviará as informações ao host. 1.7 = 70% de aumento.
float busVoltagePreDefinida = 6;
bool resetarPosicaoDiaria = true;       //Auxilia a estrutura a voltar para a posição 25 horizontal para o inicio de um novo dia.

/* Variáveis do Sol */
double horarioSolar = 0.0;
double declinacaoSolar = 0.0;
double zenite = 0.0;

/* Variásveis para módulo SD card */
#define SD_CS 5                         //Definir pino CS 
int readingID = 0;                      //Contagem da leitura para salvar na memória RTC. Esse valor é incrementado cada vez que o loop de análise roda
String dadosMensagem;                   //Variavel para ser enviada. Padrão da variavel é a string: "AnguloHorizontal/AnguloVertical"
String dadosAcopladosMensagem;          //Mensagem que será acoplada na que será enviada para o host.

/* Credenciais da rede de WIFI e Host */
const char *ssid = "";     //Nome da Rede Wifi
const char *password = "";  //Senha da Rede Wifi
WiFiClient client;                      //Declara o cliente
WiFiUDP udp;                            //Cria um objeto da classe UDP.
IPAddress server(192,168,1,184);        //Fixar o IP do host
int portaServidor = 555;                //Declarar porta de comunicação que o host irá disponibilizar

/* Definição do NTPClient e Variáveis para salvar informações sobre data e hora. */
NTPClient timeClient(udp);
// Definir o valor do offset em segundos para ajustar a timezone, por exemplo:
// GMT +1 = 3600
// GMT +8 = 28800
// GMT -1 = -3600
// GMT 0 = 0
int gmt_Correto = -10800;
String formattedDate;
String dayStamp;
String timeStamp;

/* Variáveis da análise da estrutura */
Servo servo1;                          //Servo Horizontal
Servo servo2;                          //Servo Vertical
int velocidade_Servos = 30;
int pino_Servo_Horizontal = 13;        //Localização do pino Horizontal
int pino_Servo_Vertical = 14;          //Localização do pino Vertical

int angulo_Vertical = 0;               //Variável para guardar angulo vertical atual
int angulo_Horizontal = 0;             //Variável para guardar angulo horizontal atual
int previous_Angulo_Horizontal = 0;    //Variável para guardar angulo horizontal anterior
int previous_Angulo_Vertical = 0;      //Variável para guardar angulo vertical anterior

int ldr_cimaesquerda = 33;             //Pino do LDR de Cima/Esquerda \\ROXO
int ldr_cimadireita = 32;              //Pino do LDR de Cima/Direita  \\AZUL
int ldr_baixoesquerda = 34;            //Pino do LDR de Baixo/Esquerda  \\VERDE
int ldr_baixodireita = 35;             //Pino do LDR de Baixo/Direita \\CINZA
float tolerancia = 1.05;               //Utilizado para calibrar a tolerancia entre as médias dos valores de Resistencia/Lux dos LDRs. Valor em porcentagem de erro. Ex: 1.05 = 5% de erro

bool tudoCertoHorizontal = false;      //Auxiliar do Loop para Obter valores dos LDRs. Caso haja variação dos angulos verticais, devemos analisar os horizontais novamente.
bool tudoCertoVertical = false;        //Auxiliar do Loop para Obter valores dos LDRs. Caso haja variação dos angulos horizontais, devemos canalisar os verticais novamente.
float cimaesquerda;                    // Valor de leitura LDR Cima/Esquerda
float cimadireita;                     // Valor de leitura LDR Cima/Direita
float baixoesquerda;                   // Valor de leitura LDR Baixo/Esquerda
float baixodireita;                    // Valor de leitura LDR Baixo/Direita
float media_cima;                      // Média dos LDRs superiores
float media_baixo;                     // Média dos LDRs inferiores
float media_esquerda;                  // Média dos LDRs esquerdos
float media_direita;                   // Média dos LDRs direitos
float diff_cimabaixo;                  // Diferença das médias CIMA/BAIXO
float diff_esquerdadireita;            // Diferença das médias ESQUERDA/DIREITA

/* Variáveis do INA219 (Obtenção de corrente e tensão)*/
Adafruit_INA219 ina219;

static float previous_shuntvoltage = 0;     //Tensão Shunt anterior.
static float previous_busvoltage = 0;       //Tensão Bus Anterior.
static float previous_current_mA = 0;       //Corrente em mA anterior.
static float previous_loadvoltage = 0;      //Tensão de carga anterior.
static float previous_power = 0;            //Potência anterior.

static float shuntvoltage = 0;              //Tensão Shunt.
static float busvoltage = 0;                //Tensão Bus.
static float current_mA = 0;                //Corrente em mA.
static float loadvoltage = 0;               //Tensão de carga.
static float power = 0;                     //Potência.

static float shuntvoltage_EF = 0;           //Tensão Shunt da Estrutura em posição "Fixa".
static float busvoltage_EF = 0;             //Tensão Bus da Estrutura em posição "Fixa".
static float current_mA_EF = 0;             //Corrente em mA da Estrutura em posição "Fixa".
static float loadvoltage_EF = 0;            //Tensão de carga da Estrutura em posição "Fixa".
static float power_EF = 0;                  //Potência da Estrutura em posição "Fixa".

//* Variáveis para simulação da estrutura fixa
int anguloFixoHorizontal = 90;              //Apontado para o norte.
int anguloFixoVertical = 22;               //Latitude do Rio de Janeiro 22 graus sul. 

/* Estrutura para representar uma DataHora */
typedef struct{
    int hora;
    int minuto;
    int segundo;
    int dia;
    int mes;
    int ano;
}MinhaDataHora;

MinhaDataHora tempoAtual;

//================================================================ ESTÁGIO PRINCIPAL
void setup(){
  Serial.begin(115200);                     //Inicialização do console.
  Wire.begin();                             //Inicializa a comunicação I2C.
  
  if (!ina219.begin()) {
    Serial.println("Falha ao encontrar o chip do INA219.");
    while (!ina219.begin()) {
      Serial.print(".");
      delay(1000);
    }
  }
  else
  {
    Serial.println("Sucesso ao inicializar INA219.");
  }  

  WiFi.mode(WIFI_STA);                      //Define o ESP como Station.
  servo1.attach(pino_Servo_Horizontal);     //Atacha o servo horizontal com o pino correspondente da comunicação pwm.
  servo2.attach(pino_Servo_Vertical);       //Atacha o servo vertical com o pino correspondente da comunicação pwm.
  
  /* Inicializar os Servos de forma suave para a posição 90º horizontal e 45º vertical. */
  horizontalServoMove(90, velocidade_Servos);
  verticalServoMove(tratarAnguloVerticalReferencialMotor(45), velocidade_Servos);
  wifiConnect();                            //Realiza a conexão wifi
    
  timeClient.begin();                       //Inicia o TimeClient para obter Hora e Data correta.
  timeClient.setTimeOffset(gmt_Correto);    //Set no valor de GMT de acordo com a localização terrestre.
  pinMode(ledPin, OUTPUT);                  //Declara o pino do LED de comunicação.
}

void loop(){
  if(ativarBloqueioPeloHorario){
    Serial.println("Bloqueio por horário ativado...");
    getTimeStamp();
    if(tempoAtual.hora >= 7 && tempoAtual.hora < 19){
      mainFunction();
    }
    else{
      if(tempoAtual.hora >= 19 || tempoAtual.hora < 7){
        if(resetarPosicaoDiaria){
          horizontalServoMove(45, velocidade_Servos);
          verticalServoMove(tratarAnguloVerticalReferencialMotor(45), velocidade_Servos);
          resetarPosicaoDiaria = false;
        }
      }
      else{
        resetarPosicaoDiaria = true;
      }
    }
    delay(60000);
  }
  else{
    mainFunction();
  }
}

//================================================================= FUNÇÕES AUXILIARE/* Função Main de funcionamento */
void mainFunction(){

    if (WiFi.status() != WL_CONNECTED)
    {
      wifiConnect();
    }
  
    getCurrentTension();
    previous_shuntvoltage = shuntvoltage;
    previous_busvoltage = busvoltage;
    previous_current_mA = current_mA;
    previous_loadvoltage = loadvoltage;
    previous_power = power;
    
    previous_Angulo_Horizontal = servo1.read();   //angulo_Horizontal;
    previous_Angulo_Vertical = servo2.read();     //angulo_Vertical;
  
    getReadings();
    delay(1000);                                  //Delay antes de medir a tensao e corrente novamente
    getCurrentTension();                          //Obtem, novamente, os valores de corrente e tensão após a movimentação
    getTimeStamp();                               //Função para pegar data e hora do NTPClient
  
    resetPosicaoFixa();
    calcularPosicaoSolar();
  
    if(busvoltage >= previous_busvoltage*tolerancia_Voltagem)
    {
      Serial.println("Iniciar envio de informações: Tensão bus superior à tolerância estabelecida.");
      enviarInfos("1");
    }
    else
    {
       Serial.println("Tensão bus inferior à tolerância estabelecida.");
       enviarInfos("0");
    }
    
    readingID++;
    Serial.println("Análise concluída! Uma nova análise será realizada em " + String(loopTime / 1000) + " segundos");
    delay(loopTime);
}

/* Função para movimentação do servo horizontal */
void horizontalServoMove(int angulo, int velocidade){
  int posicaoH = servo1.read();
  if(angulo > posicaoH){
    for(int auxiliar = posicaoH; auxiliar <= angulo; auxiliar += 1)
    {
      //servo1.attach(pino_Servo_Horizontal);
      servo1.write(auxiliar);
      //posicaoH = auxiliar;
      delay(velocidade);
      //servo1.detach();
    }
  }
  else{
      for(int auxiliar = posicaoH; auxiliar >= angulo; auxiliar -= 1)
      {
        //servo1.attach(pino_Servo_Horizontal);
        servo1.write(auxiliar);
        //posicaoH = auxiliar;
        delay(velocidade);   
        //servo1.detach();
      }
  }
}

/* Função para movimentação do servo vertical */
void verticalServoMove(int angulo, int velocidade){
  int posicaoV = servo2.read();
  if(angulo > posicaoV){
    for(int auxiliar = posicaoV; auxiliar <= angulo; auxiliar += 1)
    {
      servo2.write(auxiliar);
      delay(velocidade);
    }
  }
  else{
      for(int auxiliar = posicaoV; auxiliar >= angulo; auxiliar -= 1)
      {
      servo2.write(auxiliar);
      delay(velocidade);   
      }
  }
}

/* Colocar angulo vertical no referencial do servo vertical */
int tratarAnguloVerticalReferencialMotor(int angulo){
  //Para o referencial do motor, considera-se 0° como 89° do motor e 90° como 179° do motor.
  //Adaptação feita devido ao posicionamento do motor na estrutura e ao seu eixo de giro.
  //Logo, com interpolação simples:
  return 179 - angulo;
}

/* Simular placa de posicionamento fixo */
void resetPosicaoFixa(){
  Serial.println("==============");
  Serial.println("Movimento para a posição para simular a placa fixa.");
  horizontalServoMove(anguloFixoHorizontal, velocidade_Servos);
  verticalServoMove(tratarAnguloVerticalReferencialMotor(anguloFixoVertical),velocidade_Servos);
  delay(1000);
  Serial.println("Aferindo os valores de tensão, corrente e potência.");
  shuntvoltage_EF = ina219.getShuntVoltage_mV();
  busvoltage_EF = ina219.getBusVoltage_V();
  current_mA_EF = ina219.getCurrent_mA();
  loadvoltage_EF = busvoltage_EF + (shuntvoltage_EF / 1000);
  power_EF = busvoltage_EF * current_mA_EF;
  //Volta para a posição da análise
  delay(1000);
  Serial.println("Movimento para a posição analisada.");
  horizontalServoMove(angulo_Horizontal, velocidade_Servos);
  verticalServoMove(angulo_Vertical,velocidade_Servos);
  Serial.println("==============");

}

/* Sub-rotina para enviar dados ao host. */
void enviarInfos(String moverOuNao){
  //Passar para os referenciais com o solo;
  previous_Angulo_Vertical = (179-previous_Angulo_Vertical);
  angulo_Vertical = (179-angulo_Vertical);
  Serial.print("Ângulo Horizontal a ser enviado: ");
  Serial.println(angulo_Horizontal);
  Serial.print("Ângulo Vertical a ser enviado: ");
  Serial.println(angulo_Vertical);

  
  if (WiFi.status() == WL_CONNECTED) //Só irá enviar dados se estiver conectado.
  {
    digitalWrite(ledPin, HIGH);      //Acender LED apenas para mostrar que está começando a comunicação.
    Serial.println("\nIniciando conexão com o host...");
    if (client.connect(server, portaServidor)){
      Serial.println("Enviando informações para o host...");
      dadosAcopladosMensagem = String(readingID) + "," + String(dayStamp) + "," + String(timeStamp) + "," +
                  String(previous_Angulo_Vertical) + "," + String(angulo_Vertical) + "," +
                  String(previous_Angulo_Horizontal) + "," + String(angulo_Horizontal) + "," +
                  String(horarioSolar) + "," + String(declinacaoSolar) + "," + String(zenite) + "," +
                  String(previous_busvoltage) + "," + String(busvoltage) + "," +
                  String(previous_shuntvoltage) + "," + String(shuntvoltage) + "," +
                  String(previous_current_mA) + "," + String(current_mA) + "," +
                  String(cimaesquerda) + "," + String(cimadireita) + "," + String(baixoesquerda)+ "," + String(baixodireita) + "," +
                  String(shuntvoltage_EF) + "," + String(busvoltage_EF) + "," + 
                  String(current_mA_EF) + "," + String(loadvoltage_EF) + "," + String(power_EF) + "\r\n";
                  
      dadosMensagem = String(moverOuNao) + "*" + String(angulo_Horizontal) + "/" + String(angulo_Vertical) + "-" + dadosAcopladosMensagem;
      client.println(dadosMensagem);
      //client.flush();
      Serial.println("=============================");
      Serial.println("Informações enviadas: ");
      Serial.println(dadosMensagem);
      Serial.println("=============================");
      digitalWrite(ledPin, LOW);
    }
    else{
        Serial.println("Sem host conectado...");
        delay(250);
    }
  }
  else
  {
    Serial.println("Sem conexão com a rede...");
    delay(250);
  }
}

/* Capturar valores de Corrente e Tensão */
void getCurrentTension(){
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  power = busvoltage * current_mA;

  Serial.print("Tensão Bus: "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Tensão Shunt: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Tensão de Carga: "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Corrente: "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Potência: "); Serial.print(power); Serial.println(" mW");
  Serial.println("");
}

/* Função para obter novos ângulos */
void getReadings(){
  angulo_Horizontal = servo1.read();
  angulo_Vertical = servo2.read();
  
  //Inicialização para console
  Serial.println("================ INÍCIO ================");
  Serial.print("Angulo Vertical: ");
  Serial.println(angulo_Vertical);
  Serial.print("Angulo Horizontal: ");
  Serial.println(angulo_Horizontal);

  tudoCertoHorizontal = false;
  tudoCertoVertical = false;
  cimaesquerda = 0;
  cimadireita = 0;
  baixoesquerda = 0;
  baixodireita = 0;
    
  int contadorCombinacaoMaxima = 0;                //Auxiliar para sair do While caso os servos travem em loop infinito no range do ângulo desejado
  float tensao1,tensao2,tensao3,tensao4,lum1,lum2,lum3,lum4;

  while (tudoCertoHorizontal == false || tudoCertoVertical == false)
  {
    contadorCombinacaoMaxima++;
    if(contadorCombinacaoMaxima >= 16200)
    {
      break;
    }
    cimaesquerda = analogRead(ldr_cimaesquerda);   //Leitura LDR Cima/Esquerda
    cimadireita = analogRead(ldr_cimadireita);     //Leitura LDR Cima/Direita
    baixoesquerda = analogRead(ldr_baixoesquerda); //Leitura LDR Baixo/Esquerda
    baixodireita = analogRead(ldr_baixodireita);   //Leitura LDR Baixo/Direita

    tensao1 = (cimaesquerda/4095.0)*3.3;
    tensao2 = (cimadireita/4095.0)*3.3;
    tensao3 = (baixoesquerda/4095.0)*3.3;
    tensao4 = (baixodireita/4095.0)*3.3;

    //lum1 = 177.61*pow(tensao1,-1.445);
    lum1 = 586.97*pow(tensao1,-0.885);
    lum2 = 451.51*pow(tensao2,-1.002);
    lum3 = 476.21*pow(tensao3,-1.185);
    lum4 = 520.43*pow(tensao4,-0.92);

    Serial.println("================ LUX ================");
    Serial.print("Cima Esquerda:");
    Serial.println(lum1);
    Serial.print("Cima Direita:");
    Serial.println(lum2);
    Serial.print("Baixo Esquerda:");
    Serial.println(lum3);
    Serial.print("Baixo Direita:");
    Serial.println(lum4);

    //Variáveis para sabermos quando sair do 'Loop' de análise dos ângulos
    Serial.println("================ LDRs ================");
    Serial.print("Cima Esquerda:");
    Serial.println(cimaesquerda);
    Serial.print("Cima Direita:");
    Serial.println(cimadireita);
    Serial.print("Baixo Esquerda:");
    Serial.println(baixoesquerda);
    Serial.print("Baixo Direita:");
    Serial.println(baixodireita);

    /*
    media_cima = (cimaesquerda + cimadireita) / 2;
    media_baixo = (baixoesquerda + baixodireita) / 2;
    media_esquerda = (cimaesquerda + baixoesquerda) / 2;
    media_direita = (cimadireita + baixodireita) / 2;
    */

    media_cima = (lum1 + lum2) / 2;
    media_baixo = (lum3 + lum4) / 2;
    media_esquerda = (lum1 + lum3) / 2;
    media_direita = (lum2 + lum4) / 2;

    Serial.println("================ MÉDIAS ================");
    Serial.print("Cima:");
    Serial.println(media_cima);
    Serial.print("Baixo:");
    Serial.println(media_baixo);
    Serial.print("Esquerda:");
    Serial.println(media_esquerda);
    Serial.print("Direita:");
    Serial.println(media_direita);
    
    diff_cimabaixo = (media_cima - media_baixo);
    diff_esquerdadireita = (media_esquerda - media_direita);

    Serial.println("================ DIFERENÇAS ================");
    Serial.print("Diferença das médias Cima/Baixo:");
    Serial.println(diff_cimabaixo);
    Serial.print("Diferença das médias Esquerda/Direita:");
    Serial.println(diff_esquerdadireita);

    float razao_cima_baixo = media_cima/media_baixo;
    float razao_esquerda_direita = media_esquerda/media_direita;

    Serial.println("================ RAZÕES ================");
    Serial.print("Razao das médias Cima/Baixo:");
    Serial.println(razao_cima_baixo);
    Serial.print("Razao das médias Esquerda/Direita:");
    Serial.println(razao_esquerda_direita);
    Serial.println("========================================");

    //Inicialização do movimento com o servo motor levando em consideração a medida dos LDRs
    //if (diff_cimabaixo > tolerancia || -1 * diff_cimabaixo > tolerancia) // Verificação Vertical
    if (razao_cima_baixo > tolerancia ||  pow(razao_cima_baixo,-1) > tolerancia) // Verificação Vertical
    {
      //Se a diferença for maior que a tolerância, significa que os LDRs estão medindo valores muito distintos. O servo responsável pelo movimento vertical deverá se movimentar.
      if (media_cima > media_baixo) //Se a média de cima for maior que a de baixo, significa que temos menor incidencia luminosa nos resistores superiores.
      {
        angulo_Vertical = angulo_Vertical + 1;
        if (angulo_Vertical > 179)
        {
          angulo_Vertical = 179;
        }
      }
      else if (media_cima < media_baixo)
      {
        angulo_Vertical = angulo_Vertical - 1;
        if (angulo_Vertical < 89)
        {
          angulo_Vertical = 89;
        }
      }
      Serial.print("Angulo Vertical: ");
      Serial.println(angulo_Vertical);
      
      //servo2.write(angulo_Vertical);
      verticalServoMove(angulo_Vertical, velocidade_Servos);
      tudoCertoHorizontal = false;
    }
    else
    {
      tudoCertoVertical = true;
    }

    //if (diff_esquerdadireita > tolerancia || -1 * diff_esquerdadireita > tolerancia) // Verificação Horizontal
    if (razao_esquerda_direita > tolerancia ||  pow(razao_esquerda_direita,-1) > tolerancia)
    {
      //Se a diferença for maior que a tolerância, significa que os LDRs estão medindo valores distintos. O servo responsável pelo movimento horizontal deverá se movimentar.
      if (media_esquerda < media_direita) //Se a média da direita for menor que a da esquerda, significa que temos maior incidencia luminosa nos resistores da direita.
      {
        angulo_Horizontal = angulo_Horizontal + 1;
        if (angulo_Horizontal > 180)
        {
          angulo_Horizontal = 180;
        }
      }
      else if (media_esquerda > media_direita)
      {
        angulo_Horizontal = angulo_Horizontal - 1;
        if (angulo_Horizontal < 0)
        {
          angulo_Horizontal = 0;
        }
      }
      Serial.print("Angulo Horizontal: ");
      Serial.println(angulo_Horizontal);
      
      //servo1.write(angulo_Horizontal);
      horizontalServoMove(angulo_Horizontal, velocidade_Servos);
      tudoCertoVertical = false;
    }
    else
    {
      tudoCertoHorizontal = true;
    }
  }
  
  Serial.print("Ângulo Horizontal Final (Referencial do Motor): ");
  Serial.println(angulo_Horizontal);
  Serial.print("Ângulo Vertical Final (Referencial do Motor): ");
  Serial.println(angulo_Vertical);
}

/* Função para obter data e hora do NTPClient */
void getTimeStamp(){
  while (!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  // A data virá neste formato:
  // 2018-05-28T16:00:13Z
  // Abaixo iremos reformatar
  formattedDate = timeClient.getFormattedDate();

  // Extrair data
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.println(dayStamp);

  // Extrair Hora
  timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  Serial.println(timeStamp);

  //Atualizar objeto tempoAtual com as informações.
  int hora, minuto, segundo, dia, mes, ano;
  splitT = formattedDate.indexOf(":");
  hora = timeStamp.substring(0, splitT).toInt();
  minuto = timeStamp.substring(3, 5).toInt();
  segundo = timeStamp.substring(6, 8).toInt();

  splitT = formattedDate.indexOf("-");
  ano = dayStamp.substring(0, splitT).toInt();
  mes = dayStamp.substring(5 , 7).toInt();
  dia = dayStamp.substring(8, 10).toInt();

  tempoAtual.hora = hora;
  tempoAtual.minuto = minuto;
  tempoAtual.segundo = segundo;
  tempoAtual.dia = dia;
  tempoAtual.mes = mes;
  tempoAtual.ano = ano;
}

/* Função para conectar ao Wifi local */
void wifiConnect(){
  // Conecção em rede Wi-Fi com SSID e senha
  Serial.print("Conectando em ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado.");
}

//================================================================= FUNÇÕES DA POSIÇÃO DO SOL EM DETERMINADO DIA DO ANO
void calcularPosicaoSolar(){
  //Calcular angulo Zenite e Horário Solar
  horarioSolar = calculoHorarioSolar(tempoAtual.hora, tempoAtual.minuto);
  declinacaoSolar = calculoDeclinacaoSolar();
  zenite = calculoZenite(horarioSolar, declinacaoSolar, float(-1*anguloFixoVertical));

  Serial.print("Ângulo Horário Solar: "); Serial.println(horarioSolar);
  Serial.print("Ângulo Declinação Solar: "); Serial.println(declinacaoSolar);
  Serial.print("Ângulo Zenital: "); Serial.println(zenite);
}
/* Função para calcular a angulação zenite */
double calculoZenite(double horarioSolar, double declinacaoSolar, float latitude){
  double zeniteRadianos = acos(cos(declinacaoSolar*DEG_TO_RAD)*cos(horarioSolar*DEG_TO_RAD)*cos(latitude*DEG_TO_RAD)+sin(declinacaoSolar*DEG_TO_RAD)*sin(latitude*DEG_TO_RAD));
  return zeniteRadianos*RAD_TO_DEG;
}

/* Função para calcular a declinação Solar */
double calculoDeclinacaoSolar(){
  int diaN = diaDoAno(tempoAtual.ano, tempoAtual.dia, tempoAtual.mes, tempoAtual.hora, tempoAtual.minuto, tempoAtual.segundo) + 1;
  double declinacaoSolarRadianos = asin(-1*sin(23.45*DEG_TO_RAD)*cos((360/365.25)*(diaN+10)*DEG_TO_RAD));
  return declinacaoSolarRadianos*RAD_TO_DEG;
}

/* Função para calcular a angulação do Horário Solar */
double calculoHorarioSolar(int hora, int minuto){
  if(hora < 12)
  {
    return -180+(hora*15)+minuto*0.25;
  }
  else
  {
    if(hora > 12)
    {
      return (hora-12)*15 + minuto*0.25;
    }
    else
    {
      if(hora == 12 && minuto == 0)
      {
        return 0;
      }
      else
      {
        return minuto*0.25;
      }
    }
  }
  //Considerando 06h como sendo 0° e 18h como sendo 180°
}

/* Função que cacula o dia do ano entre 0 e 365 */
int diaDoAno(int anoCorrente, int dia, int mes, int hora, int minuto, int segundo){
    MinhaDataHora d1, d2;
    d1.hora = 0;
    d1.minuto = 0;
    d1.segundo = 0;
    d1.dia = 1;
    d1.mes = 1;
    d1.ano = anoCorrente;
    
    d2.hora = hora;
    d2.minuto = minuto;
    d2.segundo = segundo;
    d2.dia = dia;
    d2.mes = mes;
    d2.ano = anoCorrente;

  return calculaDiferencaEmDias(d1, d2);
}

/* Função para verificação se um ano é bissexto */
bool verificarBissexto(int ano){
    /*
      Um ano só é bissexto se ele for divisível por 4 e NÃO FOR divisível por 100
      (a não ser que ele seja divisível por 100 E TAMBÉM divisível por 400).
    */
    return (ano % 4 == 0) && (ano % 100 != 0 || ano % 400 == 0);
}

/* Função para contar quantos anos bissextos tem entre o ano 1/1/1 e o ano dado */
int contaBissextos(int ano){
    return (ano / 4) - (ano / 100) + (ano / 400);
}

/* Função para converter a DataHora dada para segundos */
unsigned long long dataParaSegundos(MinhaDataHora d){
    unsigned long long total = 0LL;

    /*
     Primeiro, calcula o tempo decorrido em segundos até a data
     sem contar os anos bissextos, considerando:

     31536000 = número de segundos em um ano não bissexto (365 dias)
     86400 = número de segundos em um dia (24 horas)
     3600 = número de segundos em 1 hora (60 minutos)
     60 = número de segundos em 1 minuto (60 segundos)
    */

    total += (d.ano - 1) * 31536000LL;

    int meses[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    for(int mes = 1; mes < d.mes; mes++)
        total += meses[mes-1] * 86400LL;

    total += (d.dia - 1) * 86400LL;

    total += d.hora * 3600LL;

    total += d.minuto * 60LL;

    total += d.segundo;

    /*
     Então, adiciona segundos para cada dia adicional dos anos bissextos
    */

    /* Número de dias adicionais, para os anos bissextos anteriores ao ano atual */
    int diasExtras = contaBissextos(d.ano - 1);
    total += diasExtras * 86400LL;

    /* Se o mês da data já passou de fereveiro e o ano atual é bissexto, adiciona mais um dia */
    if(verificarBissexto(d.ano) && (d.mes - 1) >= 2)
        total += 86400LL;

    return total;
}

/* Função para o cálculo da diferença em segundos entre duas datas */
unsigned long long calculaDiferencaEmSegundos(MinhaDataHora d1, MinhaDataHora d2){
    unsigned long long segundosd1 = dataParaSegundos(d1);
    unsigned long long segundosd2 = dataParaSegundos(d2);

    if(segundosd1 > segundosd2)
        return segundosd1 - segundosd2;
    else
        return segundosd2 - segundosd1;
}

/* Função para o cálculo da diferença em dias entre duas datas */
unsigned long long calculaDiferencaEmDias(MinhaDataHora d1, MinhaDataHora d2){
    unsigned long long segundos = calculaDiferencaEmSegundos(d1, d2);
    return segundos / 86400LL;
}
