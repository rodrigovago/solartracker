#include <ESP32Servo.h>        //Necessário para controlar servomotor
#include <WiFi.h>              //Necessário para conexão com rede wifi.
#include <NTPClient.h>         //Necessário para pegar o horário de Brasília
#include <WiFiUdp.h>           //Biblioteca do UDP utilizada pelo NTPCliente para conectar na internet.
#include <ESPAsyncWebServer.h> //Biblioteca para criar um servidor assincrono de informações.
#include "SPIFFS.h"            //Biblioteca que permite armazenamento de documentos dentro da memória do esp32. Assim que iremos hospedar um "index.html" dentro dele.
#include "FS.h"                //FileSystem - Permite o controle de documentos. (Criar, editar, apagar etc)
#include <Wire.h>              //Necessário para INA219 (I2C/SDA//SCL)
#include <Adafruit_INA219.h>   //INA219
#include "SD.h"                //Biblioteca para utilização do módulo cartão sd.
#include "SPI.h"               //Biblioteca Serial. Utilizada para comunicação com outros dispositivos. Utilizam as portas MISO/MOSI/SCK

//=============  Variáveis do Servo Motor
Servo servo1;                   //Servo Horizontal
Servo servo2;                   //Servo Vertical
int pinoServo1 = 2;             //Motor Horizontal
int pinoServo2 = 15;             //Motor Vertical
int velocidade_ServoMotor = 0; //Tempo em ms de espera para variação de um angulo para o outro.     
int Horizontal = 0;              //Angulo Horizontal que vem do rastreador.
int Vertical = 0;                //Angulo Vertical que vem do rastreador
int ultimoVertical = 0;
int ultimoHorizontal = 0;

//=============  Variáveis para módulo SD card
#define SD_CS 5                  //Definir pino CS
int readingID = 0; //Contagem da leitura para salvar na memória RTC. Esse valor é incrementado cada vez que o loop de análise roda
String dataMessage; //Mensagem que será acoplada na que será enviada para o host.

//============= Variáveis do servidor assincrono
WiFiServer hostServer(555);
WiFiClient client;
WiFiUDP udp;                          //Cria um objeto da classe UDP.
IPAddress local_IP(192, 168, 1, 184); //Setar ip fixo: 192.168.1.184
IPAddress gateway(192, 168, 1, 1);    //Setar gateway padrão: 192.168.1.1
IPAddress subnet(255, 255, 255, 0);   //Configurações de Mascara de SubRede.
IPAddress primaryDNS(8, 8, 8, 8);     //Configuração de dns primário.
IPAddress secondaryDNS(8, 8, 4, 4);   //Configuração de dns secundário.
AsyncWebServer server(80);            //Criar um AsyncWebServer na porta 80

//============= Definição do NTPClient e Variáveis para salvar informações sobre data e hora.
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

//============= Variáveis do INA219 (Obtenção de corrente e tensão)
Adafruit_INA219 ina219;

static float previous_shuntvoltage = 0;    //Tensão Shunt anterior.
static float previous_busvoltage = 0;      //Tensão Bus Anterior.
static float previous_current_mA = 0;      //Corrente em mA anterior.
static float previous_loadvoltage = 0;     //Tensão de carga anterior.
static float previous_power = 0;           //Potência anterior.

static float shuntvoltage = 0;    //Tensão Shunt.
static float busvoltage = 0;      //Tensão Bus.
static float current_mA = 0;      //Corrente em mA.
static float loadvoltage = 0;     //Tensão de carga.
static float power = 0;           //Potência.

//============= Variáveis Gerais
int ledPin = 4;    //Pino do Led para saber se está havendo comunicação ou não.
String req;        //String que armazena os dados recebidos pela rede.
bool moverOuNao = false;
// ==================================================

void setup()
{
  Serial.begin(115200);                        //Habilita a comunicaçao serial para a string recebida ser lida no Serial monitor.
  servo1.attach(pinoServo1);                   //Pino ligado ao Servo Horizontal
  servo2.attach(pinoServo2);                   //Pino ligado ao Servo Vertival
  Vertical = 45;
  Horizontal = 90;
  ultimoVertical = 45;
  ultimoHorizontal = 90;
  verticalServoMove(Vertical);
  horizontalServoMove(Horizontal);
  connectWifi();                                //Conectar ao WiFi
  carregarComponentes();
  pinMode(ledPin, OUTPUT);                      //Declara o pino do LED de comunicação.
  
  hostServer.begin();
  //Definindo rota do servidor assincrono para / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html"); });
  server.on("/horizontal", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", readHorizontal().c_str()); });
  server.on("/vertical", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", readVertical().c_str()); });
  server.on("/tension", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", readTension().c_str()); });
  server.on("/current", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", readCurrent().c_str()); });
  server.on("/downloaddata", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SD, "/hostData.txt", String(), true); });
  server.on("/downloaddatatracker", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SD, "/data.txt", String(), true); });
  server.begin();
}

void loop()
{
  if(WiFi.status() != WL_CONNECTED)
  {
    connectWifi();
  }
  esperarInfos();
  delay(10000);
  //Funções abaixo chamadas para gravar no cartão SD, a cada 10s, os valores da hora, angulos, corrente e tensão do módulo.
  getCurrentTension();
  logSDCard();
}

void esperarInfos() //Sub-rotina que verifica se há pacotes para serem lidos.
{
    client = hostServer.available();
    if (client){
      if (client.connected())
        {
            digitalWrite(ledPin, HIGH);                     //Acende o LED para indicar que está recebendo informações
            String pacote = client.readStringUntil('\r');          //Receber informações do cliente.
            Serial.print("Informação vinda do Cliente: ");
            Serial.println(pacote);
            //client.flush();
      
            setHorizontalVertical(pacote);
            delay(500);
            
            if(moverOuNao){
              verticalServoMove(Vertical);
              delay(500);
              horizontalServoMove(Horizontal);
              delay(500);
              moverOuNao = false;
            }
            digitalWrite(ledPin, LOW);            
        }
                               
      }
      client.stop();  //Finaliza conexão com o cliente.
    delay(1000);
  
}

void carregarComponentes(){
  Serial.println("Inicializando os componentes do sistema.");
  Wire.begin();
  
  Serial.println("== SD Card ==");
  //Inicializar o SDCard
  int contagemSD = 1;
  while (!getSDCard())
  {
    if(contagemSD >= 30)
    {
      break;
    }
    else
    {
      Serial.print("Tentativa de nº "); Serial.print(contagemSD); Serial.println("/30.");
      delay(2000);
    }
    contagemSD++;
  }
  
  Serial.println("== INA219 ==");
  if (!ina219.begin()) {
    Serial.println("Falha ao encontrar o chip do INA219.");
    while (!ina219.begin()) {
      Serial.print(".");
      delay(1000);
      }
      Serial.println("Sucesso ao inicializar INA219.");
  }
  else{
    Serial.println("Sucesso ao inicializar INA219.");
  }
  
  //Iniciar SPIFFS
  if (!SPIFFS.begin())
  {
    Serial.println("Um erro ocorreu ao tentar iniciar SPIFFS.");
    return;
  }
  else
  {
    Serial.println("Sucesso ao iniciar SPIFFS.");
  }
  
  timeClient.begin();
  timeClient.setTimeOffset(gmt_Correto);    //Set no valor de GMT de acordo com a localização terrestre.  
}

//================================================================= FUNÇÕES DO SERVIDOR ASSINCRONO
String readHorizontal() {
  int horizontalValue = Horizontal;
  if (isnan(horizontalValue))
  {
    Serial.println("Falha na leitura do ângulo horizontal!");
    return "";
  }
  else
  {
    return String(horizontalValue);
  }
}
String readVertical() {
  int verticalValue = Vertical;
  if (isnan(verticalValue))
  {
    Serial.println("Falha na leitura do ângulo vertical!");
    return "";
  }
  else
  {
    return String(verticalValue);
  }
}
String readTension() {
  return String(ina219.getBusVoltage_V() + (ina219.getShuntVoltage_mV()/1000));
}
String readCurrent() {
  return String(ina219.getCurrent_mA());
}

//================================================================= FUNÇÕES MOVIMENTO DOS MOTORES
void verticalServoMove(int anguloVerticalRecebido) {
  //Enxerga-se um limite de angulação de 165º(Máximo para baixo) a 75º(Máximo para cima).
  //No rastreador, o ângulo máximo para cima é de 179° enquanto que o ângulo máximo para baixo é 89°.
  //Portanto, utilizando interpolação
  int posicaoVertical = 75; //Angulo do servomotor vertical. Referencialmente, foi configurado um range de 165 até 75 graus. Representando, respectivamente, os angulos 90 e 0 graus reais.
  int angulo = posicaoVertical + anguloVerticalRecebido;

  //Porém, estruturalmente, o gerador tem um limite em seu ângulo máximo para baixo de 140° para nçao colidir a estrutura.
  if(angulo > 140)
  {
    angulo = 140;
  }
  
  //int posicaoV = servo2.read();
  int posicaoV = ultimoVertical;
  int auxiliar = 0;
  if(angulo > posicaoV){
    for(auxiliar = posicaoV; auxiliar <= angulo; auxiliar += 1)
    {
      servo2.write(auxiliar);
      delay(velocidade_ServoMotor);
    }
  }
  else{
      for(auxiliar = posicaoV; auxiliar >= angulo; auxiliar -= 1)
      {
      servo2.write(auxiliar);
      delay(velocidade_ServoMotor);
      }
  }
  ultimoVertical = auxiliar;
}
void horizontalServoMove(int anguloHorizontalRecebido){
  int angulo = anguloHorizontalRecebido;
  //int posicaoH = servo1.read();

  int posicaoH = ultimoHorizontal;
  int auxiliar = 0;
  if(angulo > posicaoH){
    for(auxiliar = posicaoH; auxiliar <= angulo; auxiliar += 1)
    {
      servo1.write(auxiliar);
      delay(velocidade_ServoMotor);
    }
  }
  else{
      for(auxiliar = posicaoH; auxiliar >= angulo; auxiliar -= 1)
      {
        servo1.write(auxiliar);
        delay(velocidade_ServoMotor);
      }
  }
  ultimoHorizontal = auxiliar;
}



//================================================================= FUNÇÕES AUXILIARES
void setHorizontalVertical(String dados){
  int delimitador0 = dados.indexOf("*");
  int delimitador = dados.indexOf("/");
  int delimitador2 = dados.indexOf("-");
  String _moverOuNao = dados.substring(0, delimitador0);

  if(_moverOuNao == "0"){
    moverOuNao = false; 
  }
  else
  {
    moverOuNao = true;
  }
  
  Horizontal = dados.substring(delimitador0 + 1, delimitador).toInt();
  Vertical = dados.substring(delimitador + 1, delimitador2).toInt();

  Serial.print("Irá se mover? R: ");
  Serial.println(moverOuNao);
  Serial.print("Horizontal Tratado: ");
  Serial.println(Horizontal);
  Serial.print("Vertical Tratado: ");
  Serial.println(Vertical);

  if(!getSDCard())
  {
    Serial.println("Não foi possível armanezar as informações do rastreador no Cartão SD.");
  }
  else
  {
    String dataFromClient = dados.substring(delimitador2 + 1, dados.length()) + "\r\n";
    appendFile(SD, "/data.txt", dataFromClient.c_str()); //Juntar dados anteriores com os atuais
  }
  
}
void getTimeStamp(){
  while (!timeClient.update())
  {
    timeClient.forceUpdate();
  }
  // A data virá neste formato:
  // 2018-05-28T16:00:13Z
  // Abaixo iremos reformatar
  formattedDate = timeClient.getFormattedDate();
  //Serial.println(formattedDate);

  // Extrair data
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  Serial.println(dayStamp);
  // Extrair Hora
  timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  Serial.println(timeStamp);
}
void getCurrentTension(){
  
  previous_busvoltage = busvoltage;
  previous_shuntvoltage = shuntvoltage;
  previous_current_mA = current_mA;
  previous_loadvoltage = loadvoltage;
  previous_power = power;
  
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  power = busvoltage * current_mA;

  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power); Serial.println(" mW");
  Serial.println("");
}
bool getSDCard(){
  if (!SD.begin(SD_CS))
  {
    Serial.println("Falha ao montar SD Card.");
    SD.begin(SD_CS);
    return false;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("Não foi encontrado SD Card plugado.");
    return false;
  }
  Serial.println("Inicializando SD card...");
  if (!SD.begin(SD_CS))
  {
    Serial.println("ERROR - Falha ao inicializar SD Card!");
    return false;
  }

  // Se o arquivo 'hostData.txt' e 'data' não existir
  // Criar um arquivo no SD Card e escrever os cabeçalhos de datalogger
  File file = SD.open("/hostData.txt");
  if (!file)
  {
    Serial.println("Arquivo 'hostData' não existe.");
    Serial.println("Criando arquivo 'hostData'...");
    writeFile(SD, "/hostData.txt", "ID de Leitura, Data, Hora, Angulo Vertical, Angulo Horizontal, Voltagem Bus Anterior, Voltagem Bus Atual, Voltagem Shunt Anterior, Voltagem Shunt Atual, Corrente Anterior, Corrente Atual, Voltagem de Carga Anterior, Voltagem de Carga Atual, Potencia Anterior, Potencia Atual \r\n");
  }
  else
  {
    Serial.println("Arquivo 'hostData' já existe.");
  }
  file.close();

  File rastreadorFile = SD.open("/data.txt");
  if (!rastreadorFile)
  {
    Serial.println("Arquivo 'data' não existe.");
    Serial.println("Criando arquivo 'data'...");
    writeFile(SD, "/data.txt", "ID da Leitura, Data, Hora, Angulo Vertical Anterior, Angulo Vertical Atual, Angulo Horizontal Anterior, Angulo Horizontal Atual, Angulo Horario Solar, Angulo Declinacao Solar, Angulo Zenital, Voltagem Bus Anterior, Voltagem Bus Atual, Voltagem Shunt Anterior, Voltagem Shunt Atual, Corrente Anterior, Corrente Atual, Leitura LDR1, Leitura LDR2, Leitura LDR3, Leitura LDR4, Voltagem Shunt Placa Fixa, Voltagem Bus Placa Fixa, Corrente Placa Fixa, Voltagem de Carga Placa fixa, Potencia Placa Fixa \r\n");
  }
  else
  {
    Serial.println("Arquivo data já existe.");
  }
  rastreadorFile.close();

  return true;
}
void logSDCard(){
    getTimeStamp();
    
      dataMessage = String(readingID) + "," + String(dayStamp) + "," + String(timeStamp) + "," +
                    String(Vertical) + "," +
                    String(Horizontal) + "," +
                    String(previous_busvoltage) + "," + String(busvoltage) + "," +
                    String(previous_shuntvoltage) + "," + String(shuntvoltage) + "," +
                    String(previous_current_mA) + "," + String(current_mA) + "," + 
                    String(previous_loadvoltage) + "," + String(loadvoltage) + "," + 
                    String(previous_power) + "," + String(power) + "\r\n";
      Serial.println("Salvar dados: ");
      Serial.println(dataMessage);
    
    if(!getSDCard())
    {
      Serial.println("Não foi possível armazenar as informações do gerador no Cartão SD.");
    }
    else
    {
      appendFile(SD, "/hostData.txt", dataMessage.c_str()); //Juntar dados anteriores com os atuais
    }
    readingID++;
}
void writeFile(fs::FS &fs, const char *path, const char *message){
  Serial.printf("Escrevendo arquivo: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Falha ao abrir arquivo para gravação.");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Arquivo gravado.");
  }
  else
  {
    Serial.println("Falha na gravação.");
  }
  file.close();
}
void appendFile(fs::FS &fs, const char *path, const char *message){
  Serial.printf("Anexando ao arquivos: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("Falha ao abrir arquivo para anexar.");
    return;
  }
  if (file.print(message))
  {
    Serial.println("Dados anexados.");
  }
  else
  {
    Serial.println("Falha ao anexar.");
  }
  file.close();
}
void connectWifi(){
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) //Setar configurações padroes.
  {
    Serial.println("Falha na configuração de IP Estático.");
  }
  
  Serial.print("Conectando em ");
  Serial.println("");
  WiFi.begin("", "");
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());
}
