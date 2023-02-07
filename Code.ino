#include <Arduino.h>
#include <Wire.h>
#include "string.h"
#include "main.h"

#define TEST_CODE
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g(U8G2_R0, /* clock=*/22, /* data=*/21, /* reset=*/U8X8_PIN_NONE);
struct sensor_data sensor[NUMBER_OF_SENSORS];


String z;

//Helper C-style array of characters to store integers converted to strings
char buffer[10];

//Variáveis dos valores máx e mín para o sensor
int min_dist = 2;
int max_dist = 100;

//Variáveis de distância
int dist_step_01;
int dist_step_02;
int dist_step_03;
int dist_step_04;

//Definição dos pinos dos botões
const int BT_1 = 13;
const int BT_2 = 25;
const int BT_3 = 35;
const int BT_4 = 34;

bool L_BT_1 = 1;
bool LA_BT_1 = 1;

bool L_BT_2 = 1;
bool LA_BT_2 = 1;

bool L_BT_3 = 1;
bool LA_BT_3 = 1;

bool L_BT_4 = 1;
bool LA_BT_4 = 1;


// Funções das tarefas
void vTask_sensor(void *pvParameters);
void vTask_LCD(void *pvParameters);


// Declaração das Queues
QueueHandle_t xQueue_Sensor_LCD;
////////////////////////

/* Declare a variable of type SemaphoreHandle_t.  This is used to reference the
mutex type semaphore that is used to ensure mutual exclusive access to stdout. */
SemaphoreHandle_t xMutex;

// Protótipos das funções
void InitializePins(void *pvParameters);
void StartBuzzer(int pin);
void StopBuzzer(int pin);

//Função Setup
void setup() {
  Serial.begin(115200); // Inicia a porta série, define a transferência de data para 115200 bps
  while (!Serial); //Garante que a porta série está pronta, ates de prosseguir

  u8g.begin(); // Inicia a biblioteca u8g
  delay(100); // Interrompe o programa, durante 100ms
  u8g.setFont(u8g2_font_NokiaSmallPlain_tf);
  
  // Definição dos pinos como sendo INPUT_PULLUP 
  pinMode(BT_1, INPUT_PULLUP);
  pinMode(BT_2, INPUT_PULLUP);
  pinMode(BT_3, INPUT_PULLUP);
  pinMode(BT_4, INPUT_PULLUP);

  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1); // Garante que o setup é executado até ao fim

  /* RTOS Setup */
  // Criação da Queue
  xQueue_Sensor_LCD = xQueueCreate(10, sizeof(DataToLCD));

  /* Before a semaphore is used it must be explicitly created.  In this example
  a mutex type semaphore is created. */
  xMutex = xSemaphoreCreateMutex();

  // Criação das Tarefas
  xTaskCreatePinnedToCore(vTask_sensor, "Task Sensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTask_LCD, "Task LCD", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(InitializePins, "Task Btns", 4096, NULL, 2, NULL, 1);
  
  // Escreve uma mensagem no LCD
  Serial.println("INITIALIZED");
}

//Tarefa do sensor
void vTask_sensor(void *pvParameters) {

  TickType_t xLastWakeTime;

  /* The xLastWakeTime variable needs to be initialized with the current tick
  count.  Note that this is the only time we access this variable.  From this
  point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
  API function. */
  xLastWakeTime = xTaskGetTickCount();
  
  // Declarar as variáveis presentes nas estruturas
  DataToLCD dataToLCD;

  //Configuração dos pinos do sensor  
  dataToLCD.sensor[0].echo_pin = sensor_01_ECHO_PIN;
  dataToLCD.sensor[0].trig_pin = sensor_01_TRIG_PIN;
  dataToLCD.sensor[1].echo_pin = sensor_02_ECHO_PIN;
  dataToLCD.sensor[1].trig_pin = sensor_02_TRIG_PIN;
  dataToLCD.sensor[2].echo_pin = sensor_03_ECHO_PIN;
  dataToLCD.sensor[2].trig_pin = sensor_03_TRIG_PIN;
  
  // Configuração dos pinos do Buzzer
  dataToLCD.sensor[0].buzzer_pin = sensor_01_BUZZER;
  dataToLCD.sensor[1].buzzer_pin = sensor_02_BUZZER;
  dataToLCD.sensor[2].buzzer_pin = sensor_03_BUZZER;


   // Definir os pinos como sendo outputs ou inputs  - trigger é output, echo é input
   // Sensor 1
  pinMode(dataToLCD.sensor[0].trig_pin, OUTPUT);
  pinMode(dataToLCD.sensor[0].echo_pin, INPUT);
  // Sensor2
  pinMode(dataToLCD.sensor[1].trig_pin, OUTPUT);
  pinMode(dataToLCD.sensor[1].echo_pin, INPUT);
  // Sensor3
  pinMode(dataToLCD.sensor[2].trig_pin, OUTPUT);
  pinMode(dataToLCD.sensor[2].echo_pin, INPUT);

  // Definir os pinos do Buzzer como OUTPUT
  for (uint8_t indexBuzzer = 0; indexBuzzer < 3; indexBuzzer++) {
    pinMode(dataToLCD.sensor[indexBuzzer].buzzer_pin, OUTPUT);
  }

  dataToLCD.sensor[0].measured_distance_cm = 22;
  dataToLCD.sensor[1].measured_distance_cm = 100;
  dataToLCD.sensor[2].measured_distance_cm = 0;

  //Inicia a posição dos dados no ecrã
  //São definidos manualmente no ficheiro photoshop
  dataToLCD.sensor[0].label_startpos_x = 41;
  dataToLCD.sensor[0].label_startpos_y = 20;
  dataToLCD.sensor[0].label_endpos_x = 30;
  dataToLCD.sensor[0].label_endpos_y = 58;
  dataToLCD.sensor[1].label_startpos_x = 63;
  dataToLCD.sensor[1].label_startpos_y = 23;
  dataToLCD.sensor[1].label_endpos_x = 63;
  dataToLCD.sensor[1].label_endpos_y = 60;
  dataToLCD.sensor[2].label_startpos_x = 85;
  dataToLCD.sensor[2].label_startpos_y = 20;
  dataToLCD.sensor[2].label_endpos_x = 97;
  dataToLCD.sensor[2].label_endpos_y = 57;

  while (1) {
    
    // Medição da distância para cada sensor
    for (int i = 0; i < NUMBER_OF_SENSORS; i++) {

      //Aciona o pino por pelo menos 10 segundos
      digitalWrite(dataToLCD.sensor[i].trig_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(dataToLCD.sensor[i].trig_pin, LOW);

      //Lê o resultado
      dataToLCD.sensor[i].measured_distance_cm = pulseIn(dataToLCD.sensor[i].echo_pin, HIGH);                     // pulseIn command will measure the time for the pulse on a defined pin
      dataToLCD.sensor[i].measured_distance_cm = round(dataToLCD.sensor[i].measured_distance_cm * 0.0343 / 2.0);  // speed of sound = 343m/s, has to travel back and forth, that´s why we divide it by 2

      //Se a distância medida for inferior a 50cm, inicia o Buzzer
      if (dataToLCD.sensor[i].measured_distance_cm <= 50) {
        Serial.print("Buzzer ");
        Serial.print(i);
        Serial.println(" Started ");
        Serial.flush();
        StartBuzzer(dataToLCD.sensor[i].buzzer_pin);
      } else {
        StopBuzzer(dataToLCD.sensor[i].buzzer_pin);
      }
      // Serial.print("Sensor ");
      // Serial.print("  "); Serial.print(i); Serial.print(" ");
      // Serial.println(dataToLCD.sensor[i].measured_distance_cm);

      //Calcular o comprimento da string
      itoa(dataToLCD.sensor[i].measured_distance_cm, dataToLCD.sensor[i].buffer, 10);

      z = dataToLCD.sensor[i].buffer;
      dataToLCD.sensor[i].label_width = u8g.getStrWidth(z.c_str());
      
      // calcular as posições "label"
      dataToLCD.sensor[i].label_xpos = map(constrain(dataToLCD.sensor[i].measured_distance_cm, min_dist, max_dist), min_dist, max_dist, dataToLCD.sensor[i].label_startpos_x, dataToLCD.sensor[i].label_endpos_x);
      dataToLCD.sensor[i].label_ypos = map(constrain(dataToLCD.sensor[i].measured_distance_cm, min_dist, max_dist), min_dist, max_dist, dataToLCD.sensor[i].label_startpos_y, dataToLCD.sensor[i].label_endpos_y);
    }

    // Copy the data to the Send IN queue Struct
    xQueueSend(xQueue_Sensor_LCD, (void *)&dataToLCD, portMAX_DELAY);
    
    //Liberta o processador da tarefa corrente pelo periodo de tempo definido   
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS );
  }
}

//Tarefa do LCD
void vTask_LCD(void *pvParameters) {

  // Declarar as variáveis presentes nas estruturas
  DataToLCD dataToLCD;
  DataToLCD dataToDisplay;

//Variáveis do tipo boolean, verifica se os dados já foram recebidos  
  bool isDataReceived = false;
  bool isFirstDataReceived = false;
  String temp;
  
  TickType_t xLastWakeTime;

  /* The xLastWakeTime variable needs to be initialized with the current tick
  count.  Note that this is the only time we access this variable.  From this
  point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
  API function. */
  xLastWakeTime = xTaskGetTickCount();

  while (1) {

    //Verificar a receção dos dados
    if (xQueueReceive(xQueue_Sensor_LCD, (void *)&dataToLCD, 10) == pdTRUE) {
      isFirstDataReceived = true;
      isDataReceived = true;
    }
    
    // Se os dados foram recebidos, escrever o conteudo no dataToDisplay
    if (isDataReceived == true) {
      memcpy(&dataToDisplay, &dataToLCD, sizeof(dataToLCD));
      
      // colocar a "received flag" a "false"
      isDataReceived = false;
    }

    // Verificar se os dados foram recebidos pela primeira vez, vindo da tarefa sensor
    if (isFirstDataReceived == true) {
      u8g.firstPage();

      do {
        u8g.drawBitmap(36, 0, 56 / 8, 15, bitmap_car_image);  // Imagem do carro no topo do display
        u8g.drawBitmap(104, 0, 24 / 8, 10, bitmap_sound_on);  // Indicador de som ativo
        u8g.drawBitmap(0, 0, 24 / 8, 10, bitmap_unit_cm);     // indicador de unidades (cm) 
       
        //Obtenção do semáforo "xMutex"
        xSemaphoreTake(xMutex, portMAX_DELAY);

        //individual pieces of the distance measurement in the 3*4 grid        
        u8g.drawBitmap(24, 17, 32 / 8, 14, dataToDisplay.sensor[0].measured_distance_cm > dist_step_01 ? bitmap_sensor_01_a_on : bitmap_sensor_01_a_off);
        u8g.drawBitmap(21, 25, 32 / 8, 16, dataToDisplay.sensor[0].measured_distance_cm > dist_step_02 ? bitmap_sensor_01_b_on : bitmap_sensor_01_b_off);
        u8g.drawBitmap(18, 34, 32 / 8, 17, dataToDisplay.sensor[0].measured_distance_cm > dist_step_03 ? bitmap_sensor_01_c_on : bitmap_sensor_01_c_off);
        u8g.drawBitmap(16, 43, 32 / 8, 18, dataToDisplay.sensor[0].measured_distance_cm > dist_step_04 ? bitmap_sensor_01_d_on : bitmap_sensor_01_d_off);

        u8g.drawBitmap(48, 23, 32 / 8, 9, dataToDisplay.sensor[1].measured_distance_cm > dist_step_01 ? bitmap_sensor_02_a_on : bitmap_sensor_02_a_off);
        u8g.drawBitmap(48, 33, 32 / 8, 9, dataToDisplay.sensor[1].measured_distance_cm > dist_step_02 ? bitmap_sensor_02_b_on : bitmap_sensor_02_b_off);
        u8g.drawBitmap(47, 42, 32 / 8, 10, dataToDisplay.sensor[1].measured_distance_cm > dist_step_03 ? bitmap_sensor_02_c_on : bitmap_sensor_02_c_off);
        u8g.drawBitmap(42, 52, 40 / 8, 10, dataToDisplay.sensor[1].measured_distance_cm > dist_step_04 ? bitmap_sensor_02_d_on : bitmap_sensor_02_d_off);

        u8g.drawBitmap(72, 17, 32 / 8, 14, dataToDisplay.sensor[2].measured_distance_cm > dist_step_01 ? bitmap_sensor_03_a_on : bitmap_sensor_03_a_off);
        u8g.drawBitmap(74, 25, 32 / 8, 16, dataToDisplay.sensor[2].measured_distance_cm > dist_step_02 ? bitmap_sensor_03_b_on : bitmap_sensor_03_b_off);
        u8g.drawBitmap(77, 34, 32 / 8, 17, dataToDisplay.sensor[2].measured_distance_cm > dist_step_03 ? bitmap_sensor_03_c_on : bitmap_sensor_03_c_off);
        u8g.drawBitmap(80, 43, 32 / 8, 18, dataToDisplay.sensor[2].measured_distance_cm > dist_step_04 ? bitmap_sensor_03_d_on : bitmap_sensor_03_d_off);
        xSemaphoreGive(xMutex);

        // Para todos os sensores 
        for (int i = 0; i < NUMBER_OF_SENSORS; i++) {  
            
         // Cor preta (fundo)         
         u8g.setColorIndex(0);                        

          // desenhar uma caixa por baixo do label
          u8g.drawBox((dataToDisplay.sensor[i].label_xpos - dataToDisplay.sensor[i].label_width / 2) - 1, dataToDisplay.sensor[i].label_ypos - 2, dataToDisplay.sensor[i].label_width + 2, 8);
          
          // Converter inteiro para string do estilo C          
          itoa(dataToDisplay.sensor[i].measured_distance_cm, dataToDisplay.sensor[i].buffer, 10); 
                     
          // Cor das caixas branca
          u8g.setColorIndex(1);                                                                    
          Serial.println(dataToDisplay.sensor[i].buffer);
          Serial.flush();

          // Desenhar a distância label
          temp = dataToLCD.sensor[i].buffer;
          u8g.drawStr((dataToDisplay.sensor[i].label_xpos - dataToDisplay.sensor[i].label_width / 2) - 1, dataToDisplay.sensor[i].label_ypos - 1, temp.c_str());
        }

        // Devolução do semaforo
        xSemaphoreGive(xMutex);

      } while (u8g.nextPage());
    }
    //Liberta o processador da tarefa corrente pelo periodo de tempo definido
    vTaskDelayUntil(&xLastWakeTime, 10 / portTICK_PERIOD_MS );
  }
}

void InitializePins(void *pvParameters) {

  TickType_t xLastWakeTime;

  /* The xLastWakeTime variable needs to be initialized with the current tick
  count.  Note that this is the only time we access this variable.  From this
  point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
  API function. */
  xLastWakeTime = xTaskGetTickCount();

  // Alteração do valor máximo e mínimo lido pelos sensores
  for(;;)
  {

    // Verificar o estado do botão 1   
    L_BT_1 = digitalRead(BT_1);
    Serial.print("BTN1: ");
    Serial.println(L_BT_1);

    // Verificar se o botão 1 foi pressionado
    if (!L_BT_1 && LA_BT_1) {
      
      //Caso seja pressionado, aumenta a distância mínima em 5cm      
      min_dist = min_dist + 5;

    } else {
      // Caso não seja pressionado, valor mantém-se igual      
      min_dist = min_dist;

      LA_BT_1 = L_BT_1;
    }

    // Verificar o estado do botão 2
    L_BT_2 = digitalRead(BT_2);
    Serial.print("BTN2: ");
    Serial.println(L_BT_2);

    // Verificar se o botão 2 foi pressionado
    if (!L_BT_2 && LA_BT_2) {
      
      //Caso seja pressionado, diminui a distância mínima em 5cm      
      min_dist = min_dist - 5;

    } else {
      // Caso não seja pressionado, valor mantém-se igual      
      min_dist = min_dist;

      LA_BT_2 = L_BT_2;
    }
    
    // Verificar o estado do botão 3
    L_BT_3 = digitalRead(BT_3);
    Serial.print("BTN3: ");
    Serial.println(L_BT_3);

    // Verificar se o botão 3 foi pressionado
    if (!L_BT_3 && LA_BT_3) {

      //Caso seja pressionado, aumenta a distância máxima em 5cm 
      max_dist = max_dist + 5;

    } else {
      // Caso não seja pressionado, valor mantém-se igual      
      max_dist = max_dist;

      LA_BT_3 = L_BT_3;
    }

    // Verificar o estado do botão 4
    L_BT_4 = digitalRead(BT_4);
    Serial.print("BTN4: ");
    Serial.println(L_BT_4);

    // Verificar se o botão 4 foi pressionado    
    if (!L_BT_4 && LA_BT_4) {

      //Caso seja pressionado, diminui a distância máxima em 5cm
      max_dist = max_dist - 5;

    } else {
      // Caso não seja pressionado, valor mantém-se igual      
      max_dist = max_dist;

      LA_BT_4 = L_BT_4;
    }

    //Obtenção do semáforo "xMutex"
    xSemaphoreTake(xMutex, portMAX_DELAY);
    //Calculo da distãncia de cada "step" (quadrado/passo)
    dist_step_01 = min_dist + round((max_dist - min_dist) / 4.0 * 1.0);
    dist_step_02 = min_dist + round((max_dist - min_dist) / 4.0 * 2.0);
    dist_step_03 = min_dist + round((max_dist - min_dist) / 4.0 * 3.0);
    dist_step_04 = min_dist + round((max_dist - min_dist) / 4.0 * 4.0);
    //Devolução do semáforo "xMutex"
    xSemaphoreGive(xMutex);
    
    //Liberta o processador da tarefa corrente pelo periodo de tempo definido
    vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS );
  }
}

//Função que inicia o Buzzer
void StartBuzzer(int pin) {
  digitalWrite(pin, HIGH);
}

//Função que desliga o Buzzer
void StopBuzzer(int pin) {
  digitalWrite(pin, LOW);
}

void loop() {
  vTaskDelete(NULL);
}