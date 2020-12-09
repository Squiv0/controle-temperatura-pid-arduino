/* =================================================================================================
  ======================================Projeto de TCC==============================================
  =====Sistema de Controle de Temperatura de Combustivel Para Bancada de Teste de Motor a Diesel====
  ========================4º Semestre do Curso de Automacao Industrial==============================
  ===================================ETEC Presidente Vargas=========================================
  ======================================Dezembro de 2020============================================
  ==========================Desenvolvido por Anderson Moreira dos Anjos=============================
  ================================================================================================== */
//======================Includes========================
#include <stdlib.h>                                                                 //Biblioteca para formatação de texto 
#include <OneWire.h>                                                                //Biblioteca para acesso com apenas 1 fio a sensores
#include <DallasTemperature.h>                                                      //Biblioteca para o sensor de temperatura DS18B20
#include <Wire.h>                                                                   //Biblioteca para comunicacao I2C no Arduino
#include <LiquidCrystal_I2C.h>                                                      //Bibliotecas LCD I2C
//======================================================
//==================Atalhos dos pinos===================
#define pLigaDesliga 2                                                              //Botão que inicia ou finaliza o processo, interrupcao externa 0
#define pInterrupt 3                                                                //Interrupcao 1, circuito de deteccao de passagem por zero                  
#define pAquecedor 4                                                                //Controle do atuador da planta, saida para o triac
#define pBomba 7                                                                    //Acionamento da bomba
#define pMisturador 8                                                               //Acionamento do misturador
#define pSensorTemperatura A0                                                       //Sensor de temperatura
//======================================================
//============Declaração de variáveis globais===========
volatile long lAtuador = 0;                                                         //Variavel de lAtuador do ebulidor/ saida do controle da planta
volatile bool bEstado = true;                                                       //Variavel para mudança de estado do botão iniciar/desligar
bool bIntUmLigada = false;                                                          //Variavel de verificacao do acionamento da interrupcao 1
unsigned long tempoAtual = 0,                                                       //Variaveis de controle de tempo usando millis
              tempoPassado = 0,
              tempoAtualPID = 0,
              tempoPassadoPID = 0;
char stringTemp[5] = {""};                                                          //String para formatar a temperatura para envio serial
char sDados[20] = {""};                                                             //String para formatar o pacote de dados para envio serial
//Variaveis do PID
const int fSetPoint = 35;                                                           //SetPoint da Planta
float fErro = 0,                                                                    //Erro da planta
      fErroI = 0,                                                                   //Erro do integrativo
      fKc = 68,                                                                     //Ganho Proporcional
      fTi = 37.54,                                                                  //Tempo do integrativo
      fTd = 0.8197,                                                                 //Tempo do derivativo
      fTs = 1,                                                                      //Tempo de amostragem do sistema (1seg)
      fP = 0,                                                                       //Saida do controlador proporcional
      fI = 0,                                                                       //Saida do controlador integrativo
      fD = 0,                                                                       //Saida do controlador derivativo
      fPID = 0,                                                                     //Saida do controladr PID
      fIAnt = 0,                                                                    //Saida Anterior do controlador integrativo
      fDAnt = 0,                                                                    //Saida Anterior do controlador derivativo
      fPIDAnt = 0,                                                                  //Saida Anterior do controladr PID
      fN = 1,                                                                       //Filtro para controlador Derivativo
      fTempC = 0,                                                                   //Entrada da variavel da planta
      fTempCAnt = 0,                                                                //Entrada anterior da variavel da planta
      fTempInicial = 0,                                                             //Alimentada no inicio do processo com a temperatura ambiente
      fSatPositiva = 41,                                                            //Variavel para filtro de saturacao
      fSatNegativa = 0,                                                             //Variavel para filtro de saturacao
      fCoefAngular = 0;                                                             //Variavel para receber o coeficiente angular para conversao de temperatura
//===================================================
//===============Instanciando objetos================
OneWire oneWire(pSensorTemperatura);                                                //Instancia da classe OneWire
DallasTemperature sensors(&oneWire);                                                //Instancia da classe DallasTemperature
DeviceAddress sensorSaida;                                                          //Instancia da classe DeviceAddress
LiquidCrystal_I2C lcd(0x3F, 16, 2);                                                 //Instancia da classe LiquiCrystal_I2C com o endereco do LCD, numero de colunas e linhas
//===================================================
//================Declaração de funções==============
void zeroCross();                                                                   //Funcao acionada na interrupcao externa 1, sempre que recebe um pulso do circuito de zero crossing (a cada 8,33ms)
void ligaDesliga();                                                                 //Funcao acionada na interrupcao externa 0, ao se pressionar o pushbutton
void toggleIntUm();                                                                 //Muda o estado da variavel que controla o acionamento da interrupcao 1
void calculoPID();                                                                  //Efetua o calculo do PID
void saidaLCDOn();                                                                  //Imprime as informacoes no LCD quando o sistema esta ativo
void saidaLCDOff();                                                                 //Imprime as informacoes no LCD quando o sistema esta em stand-by
void envioSerial();                                                                 //Formata e envia um pacote de dados via RS232 para o Indusoft (para o gráfico do PID)
//==================================================

void setup() {
  Serial.begin(9600);                                                               //Inicializa a comunicação serial
  sensors.begin();                                                                  //Inicializa sensor
  lcd.init();                                                                       //Inicializa LCD
  lcd.backlight();                                                                  //Liga backlight
  lcd.clear();                                                                      //Limpa LCD
  pinMode(pAquecedor, OUTPUT);                                                      //Define pino de controle do Triac como saida
  pinMode(pBomba, OUTPUT);                                                          //Define o pino de acionamento da Bomba como saida
  pinMode(pMisturador, OUTPUT);                                                     //Define o pino de acionamento do Misturador como saida
  pinMode(pLigaDesliga, INPUT_PULLUP);                                              //Define o pino do botao de liga/desliga como input_pullup (uso do resistor interno de pull-up, acionamento em nivel logico baixo)
  attachInterrupt(digitalPinToInterrupt(pLigaDesliga), ligaDesliga, FALLING);       //Liga a interrupcao 1 (pino2), funcao de interrupcao ligaDesliga, em borda de subida

  if (!sensors.getAddress(sensorSaida, 0)) {                                        //Caso nenhum sensor seja encontrado, imprime uma mensagem de erro no monitor serial
    Serial.println("Sensores nao encontrados !");
  }
}

void loop() {
  if (bEstado) {                                                                     //Verifica estado do sistema, comeca com bEstado em false, quando true, executa o codigo principal do loop
    tempoAtual = millis();                                                           //Alimenta a variavel com o valor de millis()
    tempoAtualPID = millis();                                                        //Alimenta a variavel com o valor de millis()
    sensors.requestTemperatures();                                                   //Faz a requisicao de temperaturas da colecao de sensores
    fTempC = sensors.getTempC(sensorSaida);                                          //Salva a temperatura convertida em celsius do sensor na variavel fTempC

    if (!(fTempInicial > 0)) {                                                       //Verifica se a variavel ainda tem valor zero, garante que o bloco seja executado apenas uma vez
      fTempInicial = fTempC;                                                         //Variavel recebe o primeiro valor de leitura da temperatura para servir como temperatura ambiente
      fSatNegativa = fTempInicial;                                                   //Variavel de controle de saturacao negativa recebe primeira leitura de temperatura
      fCoefAngular = (41 - fTempInicial) / 76;                                       //Calculo para obtencao do coeficiente angular da equacao da reta para a conversao de temperatura apos PID
    }
    if (!bIntUmLigada) {                                                             //Verifica o estado da variavel e executa o bloco se for false, garante que o bloco seja executado apenas uma vez
      toggleIntUm();                                                                 //Inverte o estado da variavel de controle
      attachInterrupt(digitalPinToInterrupt(pInterrupt), zeroCross, RISING);         //Liga a interrupcao externa 1
	    
    }
    digitalWrite(pBomba, LOW);                                                       //Aciona a bomba
    digitalWrite(pMisturador, LOW);                                                  //Aciona o misturador

    if ((tempoAtualPID - tempoPassadoPID) >= 1000) {                                 //Contagem de tempo de 1seg para a execucao do calculo do PID
      tempoPassadoPID = tempoAtualPID;
      calculoPID();
    }

    saidaLCDOn();                                                                    //Executa a funcao que escreve as informacoes no LCD quando o sistema esta ativo

    if ((tempoAtual - tempoPassado) >= 200) {                                        //Contagem de tempo de 200ms para o envio de dados via Serial
      tempoPassado = tempoAtual;
      envioSerial();
    }
  }
  else {                                                                             //Executa quando estado do botao liga e desliga for false
    if (bIntUmLigada) {                                                              //Verifica o estado da variavel de controle do botao e executa o bloco caso seja false
      toggleIntUm();                                                                 //Inverte o estado da variavel de controle
      detachInterrupt(digitalPinToInterrupt(pInterrupt));                            //Para a interrupcao externa 1
    }
    digitalWrite(pBomba, HIGH);                                                      //Desliga a bomba
    digitalWrite(pMisturador, HIGH);                                                 //Desliga o misturador
    saidaLCDOff();                                                                   //Executa a funcao que escreve as informacoes no LCD quando o sistema esta em stand-by
  }
}

//=======================Funções====================
//==========Calculo da potencia do atuador==========
void zeroCross()  {
  if (lAtuador > 76) lAtuador = 76;                                                  //Limita o valor maximo da variavel de controle a 76 (menor atraso com estabilidade)
  if (lAtuador < 10) lAtuador = 10;                                                  //Limita o valor minimo da variavel de controle a 10 (evita acionamento muito proximo da proxima senoide)

  //Calculo do atraso do pulso no gate do Triac
  long t1 = 8200L * (100L - lAtuador) / 100L;                                        //Equacao para o atraso do acionamento do Triac (quanto maior o valor de lAtuador, menor o atraso)
  delayMicroseconds(t1);                                                             //Delay em microssegundo com o valor do atraso para o pulso no gate do Triac
  digitalWrite(pAquecedor, HIGH);                                                    //Pulso no gate do Triac
  delayMicroseconds(10);                                                             //Delay de 10 microssegundos
  digitalWrite(pAquecedor, LOW);                                                     //Fim do pulso no gate do Triac
}
//================================================
//===Verifica o estado da interrupt externa 0=====
void toggleIntUm() {
  bIntUmLigada = !bIntUmLigada;                                                      //Inverte o valor da variavel
}
//================================================
//=========Inicia ou encerra o sistema============
void ligaDesliga() {
  static unsigned long delayDebounce;                                                //Variavel para controle de tempo do debounce do botao
  if ((millis() - delayDebounce) > 200) {                                            //Aguarda 200ms antes de executar o bloco para evitar efeito bouncing
    bEstado = !bEstado;                                                              //Inverte o valor da variavel
    delayDebounce = millis();                                                        //Realimenta a variavel com millis para proxima verificacao
  }
}
//===============================================
//====================Calculo PID==================
void calculoPID() {
  //Calcula o erro
  fErro = fSetPoint - fTempC;                                                        //Filtro para nao estourar o integrativo - Saturacao
  if (fPIDAnt >= fSatPositiva || fPIDAnt <= fSatNegativa) {
    fErroI = 0;
  }
  else {
    fErroI = fErro;
  }
  //Calculo do PID
  fP = fKc * fErro;                                                                                                       //Calculo proporcional
  fI = fIAnt + ((fKc * fTs / fTi ) * fErroI);                                                                             //Calculo integrativo
  fD = ((fTd / (fTd + (fN * fTs))) * fDAnt) - (((fKc * fN * fTd) / (fTd + (fN * fTs))) * (fTempC - fTempCAnt));           //Calculo derivativo
  fPID = (fP + fI + fD);                                                                                                  //Soma dos calculos
  fIAnt = fI;                                                                                                             //Salva integrativo
  fDAnt = fD;                                                                                                             //Salva derivativo
  fPIDAnt = fPID;                                                                                                         //Salva PID
  fTempCAnt = fTempC;                                                                                                     //Salva temperatura

  //Filtro para diminuir esforço de controle
  if (fPID >= 41) {
    fPID = 41;
  }
  if (fPID <= fTempInicial) {
    fPID = fTempInicial;
  }
  if (fPID > fTempInicial && fPID < 41) {
    fPID = fPID;
  }  
  lAtuador = (fPID - fTempInicial) / fCoefAngular;                                                                       //Conversão para a variável do atuador
}
//=================================================
//==============Envio de dados Serial==============
void envioSerial(){
  dtostrf(fTempC, 5, 2, stringTemp);
  strcpy(sDados, "");
  strcat(sDados, "$Dados,");
  strcat(sDados, stringTemp);
  strcat(sDados, ",*");
  strcat(sDados, "\r\n");
  Serial.write(sDados);
}
//=================================================
//==================Saida do LCD On=================
void saidaLCDOn() {
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Tmp:");
  lcd.setCursor(4, 0);
  lcd.print(fTempC);
  lcd.setCursor(10, 0);
  lcd.print("Er:");
  lcd.setCursor(13, 0);
  lcd.print(fErro);
  lcd.setCursor(0, 1);
  lcd.print("At:");
  lcd.setCursor(3, 1);
  lcd.print(lAtuador);
  lcd.setCursor(5, 1);
  lcd.print(" ");
  lcd.setCursor(6, 1);
  lcd.print("PID:");
  lcd.setCursor(10, 1);
  lcd.print(fPID);
}
//=================================================
//==================Saida do LCD Off===============
void saidaLCDOff() {
  lcd.setCursor(0, 0);
  lcd.print("Pressione       ");
  lcd.setCursor(0, 1);
  lcd.print("para iniciar    ");
}
//=================================================
