/* ArDOs   v1.08a
***Дозиметр на Ардуино
***IDE Arduino 1.8.8
  ветка форума arduino.ru/forum/proekty/delaem-dozimetr
  сайт srukami.inf.ua/ardos.html
*/
#include <util/delay.h> //уже есть
#include <EEPROM.h>//уже есть
#include <LCD5110_Graph.h>//нужно установить

//настройки /////////////начало
LCD5110 myGLCD(A1, A0, 12, 10, 11); //подключение дисплея
#define contrast 60 //контрастность дисплея
//#define buzzer_active //если используется активный бузер (со встроенным генератором), управляемый транзистором с выхода 6, то раскомментировать эту строчку, если пассивный (с усилителем или без) - оставить закомментированой.
//#define UNO_DIP // если используется ArduinoUNO или плата на голой атмеге328 в корпусе DIP - раскомментируйте данную строчку. Это переключит чтение напряжения с делителя с ноги A6 на ногу A5.
#define first_alarm_duration 7000 //длительность сигнала тревоги при превышении первого аварийного порога в миллисекундах
byte treviga_1 = 30; //первая ступень тревоги
byte treviga_2 = 60; //вторая ступень тревоги
byte del_BUZZ = 7;//длительность одиночного сигнала
#define  ADC_value 185  //значение АЦП при котором 400В с учетом вашего делителя напряжения (0..255). Для значений делителя с сайта srukami 163. alexadresat 185. (Тестовая версия tekagi  67)
#define k_delitel 500 //коефициент делителя напряжения, зависит от вашего делителя. Для значений делителя с сайта srukami k_delitel 576. alexadresat 500 (Тестовая версия tekagi  1395)
byte puls = 2; //тонкая настройка длинны импульса высоковольтного транса
byte scrin_GRAF = 1; //скорость построения графика в секундах
bool podsvetka = 0; //подсветка
uint8_t graph_type = 1; //тип графика
bool alarm_sound = 0; //флаг индикации превышения порога звуком
float opornoe = 1.10; //делить на opornoe/10
#define save_DOZ 20 //как часто сохранять накопленную дозу например каждые 20мкР
#define geiger_counter_seconds 40 // число секунд для замера, соответствующее характеристикам счётчика. Для СБМ-20 равно 40.
byte beta_time = 5; //время замера бета излучения
//настройки //////////////конец
//служебные переменные
extern uint8_t SmallFontRus[], MediumNumbers[], TinyFontRus[];
volatile uint8_t timer_seconds = 0; // для отсчёта секундных интервалов в прерывании
uint8_t beta_seconds = 0;
uint8_t count_and_dose_seconds = 0;
#define maxString 21 // для работы функции преобразования кодировки utf8us
char target[maxString + 1] = ""; // для работы функции преобразования кодировки utf8us
extern uint8_t logo_bat[], logo_rag[], logo_tr[], beta_prev_1[], beta_prev_2[];
volatile int shet = 0;
int8_t ind_ON = 1;  //0 - индикация выключена, 1 - включён бузер, 2 - светодиод, 3 - и бузер, и светодиод
uint8_t first_alarm_type = 1;  //1-3, 1 - только бузер, 2 - только вибро, 3 - и бузер, и вибро
byte periodical_alarm_variable = 0; // переменная для периодически повторяющейся тревожной сигнализации
unsigned long gr_milis = 0, lcd_milis = 0;
unsigned long alarm_milis = 0; //для отсчёта длительности сигнала тревоги по превышению порога
unsigned long spNAK_milis = 0, time_doza = 0, bat_mill = 0;
uint16_t hv_adc, hv_400, shet_n = 0, shet_s = 0;
uint16_t fon = 0, fon_254 = 0;
int speed_nakT = 0, speed_nak = 0, result;
byte MIN, DAY, HOUR, MONTH; //для учёта времени дозы
uint16_t doza_vr = 0, fon_vr254 = 0, fon_vr_poisk = 0;
byte mass_p[84]; // массив для графика
byte m = 0, n_menu = 0, sys_menu = 0;
byte  mass_poisk[255]; // основной рабочий массив
byte val_kl = 0, val_ok = 0, menu = 0, zam_poisk_counter = 0;
byte sek = 0, minute = 0, bet_z = 0, gotovo = 0;
int  bet_z0 = 0, bet_z1 = 0, bet_r = 0;
float VCC = 0.0, doz_v = 0.0;
bool tr = 0, poisk = 1, fonarik = 0, toch, blink_data=1;
uint8_t GRAPH_max = 5; // максимальное значение за период отображения графика
uint8_t GRAPH_count = 0; // счётчик для поиска максимального значения для графика
#define key_pressed_left 1
#define key_pressed_right 2
#define key_pressed_ok 3
#define key_holded_left 4
#define key_holded_right 5
#define key_holded_ok 6
#define keys_not_pressed 0
uint8_t key_data = 0;
//-------------------------------------------------------------
void setup() {
  //-----------------------------------------------------------
  // настраиваем таймер на секундный интервал
  TCCR1A=(1<<WGM11); //режим14 FAST PWM 
  TCCR1B=(1<<CS12)|(1<<WGM13)|(1<<WGM12); //делить частоту CPU на 256
if (F_CPU == 16000000UL) 
 {
  ICR1=62499;  // (16000000MHz /div256) -1 = 1 раз в секунду
 }
else if (F_CPU == 8000000UL) 
 {
  ICR1=31249;  // (8000000MHz /div256) -1 = 1 раз в секунду
 } 
  TIMSK1=(1<<TOIE1); //разрешить прерывание
  //-----------------------------------------------------------
// Serial.begin(115200);
  ACSR |= 1 << ACD; //отключаем компаратор
  //ADCSRA &= ~(1 << ADEN);  // отключаем АЦП,
  pinMode(3, INPUT_PULLUP); //кнопка
  pinMode(4, INPUT_PULLUP); //кнопка
  pinMode(7, INPUT_PULLUP); //кнопка
  DDRB |= (1 << 0); PORTB &= ~(1 << 0); //пин вибры 8
  DDRC |= (0 << 4); PORTC &= ~(1 << 4); //пин пустой А4
  DDRC |= (0 << 5); PORTC &= ~(1 << 5); //пин пустой А5
  DDRB |= (1 << 1);//пин фонаря
  DDRC |= (1 << 3);//A3 дисплей GND
  DDRC |= (1 << 2);//A2 дисплей Light
  PORTC &= ~(1 << 3); //A3 дисплей GND
  PORTC  |= (1 << 2); //A2 дисплей Light
  eeprom_readS ();
  eeprom_readD ();
  lcd_init();
  attachInterrupt(0, Schet, FALLING);//прерываниям пин 2
  DDRB |= (1 << 5); //пины на выход
  DDRD |= (1 << 5);
  DDRD |= (1 << 6);
  DDRD |= (1 << 6);//пин бузера
  nakachka();
  clear_poisk_variables();
}
//-------------------------------------------------------------
void loop() 
{
key_data = get_key();  // вызываем функцию определения нажатия кнопок, присваивая возвращаемое ней значение переменной, которую далее будем использовать в коде
if (menu == 0) 
  {
    if (key_data == key_pressed_left) 	//нажатие <<<
		{
			key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду 
			clear_poisk_variables();
		}
    if (key_data == key_holded_left) //удержание <<< фонарик  
	{
		fonarik = !fonarik; 
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
	}  	
    if (key_data == key_holded_right) // удержание <<< подсветка
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
        podsvetka = !podsvetka;
    }		
    if (key_data == key_pressed_right)  //нажатие >>>
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		menu = 4;
		shet = 0;
		bet_z0 = 0;
		bet_z1 = 0;
		bet_r = 0;
		bet_z = 0;
		gotovo = 0;
		sek = 0;
		minute = 0;
    }
 } 
if (menu == 3)
  {
    if (key_data == key_pressed_left)  //нажатие <<<
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		menu = 0;
		shet = 0; fon = 0; zam_poisk_counter = 0;
		for (int i = 0; i < 18; i++) { mass_poisk[i] = 0; }//чистим
	}
  } 
if (menu == 4) 
  {
    if (key_data == key_pressed_right)  //нажатие >>>
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		menu = 0;
		clear_poisk_variables();
    }
  }
if (fonarik == 0)  //фонарик
	{
		PORTB &= ~(1 << 1);//пин фонаря
	} 
else if (fonarik == 1) 
	{
		PORTB |= (1 << 1);//пин фонаря
	}
if (podsvetka == 1) 
	{
		PORTC &= ~(1 << 2); //A2 дисплей Light
	}
else if (podsvetka == 0) 
	{
		PORTC |= (1 << 2); //A2 дисплей Light
	}
  if (millis() - lcd_milis >= 300)  //скорость отрисовки дисплея
	{
		lcd_milis = millis();
		blink_data = !blink_data;
		if (menu == 0) 
			{
				lcd_poisk();  //вывод на дисплей режима поиск
				poisk_f();    //вызов функции счёта и набора массива
			}
		if (menu == 1) 
			{
				lcd_menu();   //вывод на дисплей меню
				poisk_f();	//вызов функции счёта и набора массива 
			}
		if (menu == 2) 
			{
				lcd_sys();    //вывод на дисплей системного меню
				poisk_f();	//вызов функции счёта и набора массива
			}
		if (menu == 3) 
			{
//      		zamer_200s(); //вывод на дисплей замер 180сек
				menu = 0;
			}
		if (menu == 4) 
			{
				zamer_beta();
			}
	}
//-------------------------------------------------------------------------------------------------------------
/* Костыль. Разностный замер и длительный замер вызываются с частотой отрисовки дисплея, 
поэтому сложно поймать нажатия кнопок внутри функций zamer_200s() и zamer_beta().
Возможно в будущем придётся разделить эти функции на обработку данных (вызывать с частотой loop'а) 
и вывод на дисплей (вызывать с частотой обновления дисплея)
*/
if (menu == 4)
{
    if ((key_data == key_pressed_ok) && (gotovo == 0))  //нажатие OK
		{
			key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
			gotovo = 1;
			switch (bet_z) //проверяем, находимся ли в первом или втором замере
			{
				case 0: //если в первом замере
					bet_z0 = 0; //обнуляем текущие показания замера 1
					shet = 0; //обнуляем счёт
				case 1: //если во втором замере
					bet_z1 = 0; //обнуляем текущие показания замера 2
					shet = 0; //обнуляем счёт            
			}
		}
}
//--------------------------------------------------------------------------------------------------------------  
generator();//накачка по обратной связи с АЦП
if (shet_s != shet) 
	{
		signa ();//подача сигнала о частичке
	}
if (key_data == key_pressed_ok) { //нажатие ок
if (menu == 2) 
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		sys_menu++;
		if (sys_menu > 5) 
			{
				sys_menu = 0;
			}
	}
if (menu == 1) 
	{
		key_data = 0;    // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		n_menu++;
		if (n_menu > 7) 
			{
				n_menu = 0;
			}
	}
if (menu == 0) 
	{
		key_data = 0;   // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		menu = 1;
	}
  } 
if (menu == 0)  // в меню по удержанию кнопки "ок" входим только из режима "поиск"
	{
		if (key_data == key_holded_ok)  //удержание OK
			{
				menu = 2; 
				key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
			}
	}
if (menu == 1) 
	{
		if (key_data == key_pressed_right)  //нажатие >>>
			{
				if (n_menu == 0) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду	  
						treviga_1++;
					}
				if (n_menu == 1) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
						treviga_2++;
					}
				if (n_menu == 2) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
						podsvetka = !podsvetka;
					}
				if (n_menu == 3) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						graph_type++;
						if (graph_type>1) {graph_type = 1;}
					}
				if (n_menu == 4) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
						scrin_GRAF++;
						if (scrin_GRAF > 10) 
							{
								scrin_GRAF = 1;
							}
					}
				if (n_menu == 5) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
						ind_ON++; 
						ind_ON = constrain (ind_ON, 0, 4); //держим значение в диапазоне 0...4
					}
				if (n_menu == 6) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
						menu = 0;
					}
				if (n_menu == 7) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду		  
						eeprom_wrS ();
						menu = 0;
					}
			}
	}
if (menu == 2) 
	{
		if (key_data == key_pressed_right)  //нажатие >>>
			{
				if (sys_menu == 0) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду	
						opornoe = opornoe + 0.01;
						if (opornoe < 0.98) 
							{
								opornoe = 1.20;
							}
						if (opornoe > 1.20) 
							{
								opornoe = 0.98;
							}
					}
				if (sys_menu == 1) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						puls++;
						if (puls < 1) 
							{
								puls = 200;
							}
						else if (puls > 200) 
							{
								puls = 1;
							}
					}
				if (sys_menu == 2) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						time_doza = 0;//сброс накопленной дозы
						doz_v = 0;//сброс накопленной дозы
						eeprom_wrD ();
						myGLCD.clrScr();
						myGLCD.setFont(SmallFontRus);
						myGLCD.print(utf8rus("ДОЗА И ВРЕМЯ"), CENTER, 16);
						myGLCD.print(utf8rus("ОБНУЛЕНЫ"), CENTER, 24);
						myGLCD.update();
						_delay_ms(1000);
					}
				if (sys_menu == 3) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						menu = 0;
					}
				if (sys_menu == 4) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						eeprom_wrS ();
						menu = 0;
					}
				if (sys_menu == 5) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						beta_time++;
					}
			}
	}
if (menu == 1) 
	{
		if (key_data == key_pressed_left)  //нажатие <<<
			{
				if (n_menu == 0) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						treviga_1--;
					}
				if (n_menu == 1) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						treviga_2--;
					}
				if (n_menu == 2) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						podsvetka = !podsvetka;
					}
				if (n_menu == 3) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						graph_type--;
						if (graph_type > 1) {graph_type = 0;}
					}
				if (n_menu == 4) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						scrin_GRAF--;
						if (scrin_GRAF < 1) 
							{
								scrin_GRAF = 10;
							}
					}
				if (n_menu == 5) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						ind_ON--; 
						ind_ON = constrain (ind_ON, 0, 4); //держим значение в диапазоне 0...4
					}
				if (n_menu == 6) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						menu = 0;
					}
				if (n_menu == 7) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						eeprom_wrS ();
						menu = 0;
					}
			}
	}
if (menu == 2) 
	{
		if (key_data == key_pressed_left)  //нажатие <<<
			{
				if (sys_menu == 0) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						opornoe = opornoe - 0.01;
						if (opornoe < 0.98) 
							{
								opornoe = 1.20;
							}
						else if (opornoe > 1.20) 
							{
								opornoe = 0.98;
							}
					}
				if (sys_menu == 1) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						puls--;
						if (puls < 1) 
							{
								puls = 200;
							}
						else if (puls > 200) 
							{
								puls = 1;
							}
					}
				if (sys_menu == 2) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						time_doza = 0;//сброс накопленной дозы
						doz_v = 0;//сброс накопленной дозы
						eeprom_wrD ();
						myGLCD.clrScr();
						myGLCD.setFont(SmallFontRus);
						myGLCD.print(utf8rus("ДОЗА И ВРЕМЯ"), CENTER, 16);
						myGLCD.print(utf8rus("ОБНУЛЕНЫ"), CENTER, 24);
						myGLCD.update();
						_delay_ms(1000);
					}
				if (sys_menu == 3) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						menu = 0;
					}
				if (sys_menu == 4) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						eeprom_wrS ();
						menu = 0;
					}
				if (sys_menu == 5) 
					{
						key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
						beta_time--;
					}
			}
	}
//------------------------------------------------------------------------------------------------------------------  
if (alarm_sound && (millis() - lcd_milis >= 300)) //если поднят флаг аварийного сигнала (плюс попользуемся интервалом обновления экрана)
	{
		periodical_alarm_variable++; 
		if (periodical_alarm_variable >= 4) {periodical_alarm_variable = 1;} // держим переменную в пределах
		PORTB |= (1 << 0); //включаем вибру
		if (periodical_alarm_variable  > 1 )
			{
				PORTB &= ~(1 << 0); // выключаем вибру	
			}
		if  (periodical_alarm_variable  < 3) // периодичный звук тревоги
			{
				#ifdef buzzer_active //если задефайнен активный бузер
				PORTD |= (1 << 6); // включаем непрерывный сигнал тревоги
				#else //пассивный
				tone (6, 1300); //генерим писк с частотой 1300Гц (значение можно изменить на своё) на пине 6
				#endif
			}
		else 
			{
				#ifdef buzzer_active   //если задефайнен активный бузер
				PORTD &= ~(1 << 6); // выключаем непрерывный сигнал тревоги
				#else //пассивный бузер
				noTone (6); //выключаем писк на 6й ноге
				#endif 
			}
	if ((millis() - alarm_milis) > first_alarm_duration) // проверяем, не истекло ли время подачи сигнала тревоги
		{
			PORTB &= ~(1 << 0); // выключаем вибру
			periodical_alarm_variable = 0; // обнуляем переменную
			#ifdef buzzer_active   //если задефайнен активный бузер
			PORTD &= ~(1 << 6); // выключаем непрерывный сигнал тревоги
			#else //пассивный бузер
			noTone (6); //выключаем писк на 6й ноге
			#endif 
			alarm_sound = 0; // сбрасываем флаг сигнала тревоги
		}
	}
//------------------------------------------------------------------------------------------------------------------
if (!tr && alarm_sound) // если фон ниже порога тревоги, но сигнал тревоги ещё не выключен
	{
		res_first_alarm(); //сбрасываем сигнал тревоги
	}
}
//-------------------------------------------------------------------------------------------------------
void lcd_poisk() 
{//вывод на дисплей режима поиск
if (shet < treviga_1 && fon < treviga_1) //проверяем тревогу
	{
		tr = 0;
	}
if (shet > treviga_1 || fon > treviga_1) //проверяем тревогу
	{
		check_alarm_signal(); // устанавливаем сигнал непрерывной тревоги, если "tr" переключился в "1"
		tr = 1;
	}
myGLCD.clrScr();
myGLCD.setFont(SmallFontRus);
if (tr == 1)  //опасно
	{
		myGLCD.drawBitmap(0, 0, logo_tr, 24, 8);
	}
myGLCD.setFont(TinyFontRus);
if (fon_254 > 0) 
	{
		if (fon_254 >= 1000) 
			{
				myGLCD.print("\xBC\xBE\xBF", 43, 0);
			}
		if (fon_254 < 1000) 
			{
				myGLCD.print("\xBC\xBD\xBE\xBF", 43, 0);
			}
	}
if ((zam_poisk_counter >= 254) || blink_data)
	{
	myGLCD.setFont(TinyFontRus);
	if (fon_254 > 0) 
		{
			if (fon_254 >= 1000) 
				{
					myGLCD.printNumF((float(fon_254)/1000.0), 1, 26, 0);
				}
			if (fon_254 < 1000) 
				{
					if (fon_254 < 100)
						{
							myGLCD.printNumI(fon_254, 32, 0);
						}
					else
						{
							myGLCD.printNumI(fon_254, 26, 0);
						}
				}
		}
	}
if ((zam_poisk_counter >= geiger_counter_seconds) || blink_data)
	{
		myGLCD.setFont(MediumNumbers);
		if (fon > 0) 
			{
				if (fon >= 1000) 
					{
						myGLCD.printNumF((float(fon)/1000), 2, LEFT, 7);
						myGLCD.setFont(SmallFontRus); myGLCD.print(utf8rus("мР/ч"), RIGHT, 12);
					}
				if (fon < 1000) 
					{
						if (fon < 100)
							{
								myGLCD.printNumI(fon, CENTER, 7);
							}
						else
							{
								myGLCD.printNumI(fon, LEFT, 7);	
							}
						myGLCD.setFont(SmallFontRus); myGLCD.print(utf8rus("мкР/ч"), RIGHT, 12);
					}
			}
	}
if (fon > 0) 
	{
		myGLCD.setFont(SmallFontRus);
		if (fon >= 1000) 
			{
				myGLCD.print(utf8rus("мР/ч"), RIGHT, 12);
			}
		if (fon < 1000) 
			{
				myGLCD.print(utf8rus("мкР/ч"), RIGHT, 12);
			}
	}
time_d ();
myGLCD.setFont(TinyFontRus);
ind_doze_time();	//вывод времени накопления дозы на дисплей	  
myGLCD.setFont(SmallFontRus);
if (doz_v < 1000) 
	{
		if (doz_v < 100)
			{
				myGLCD.printNumF(doz_v, 1, 41, 24); myGLCD.print(utf8rus("мкР"), RIGHT, 24);
			}
		else
			{
				myGLCD.printNumF(doz_v, 1, 34, 24); myGLCD.print(utf8rus("мкР"), RIGHT, 24);
			}    
	}
if (doz_v >= 1000) 
	{
		myGLCD.printNumF(doz_v / 1000.0, 2, 41, 24); myGLCD.print(utf8rus("мР"), RIGHT, 24);
	}
myGLCD.drawLine(0, 32, 83, 32);//верхняя
battery();
if (graph_type == 0)
	{
	for (uint8_t i = 0; i < 82; i ++)  //печатаем график
		{
			uint8_t max_pixel = map(mass_p[i], 0, GRAPH_max, 0, 15);
			myGLCD.drawLine(i + 1, 47, i + 1, 47 - max_pixel);
		}
	}
else if (graph_type == 1)
	{
	for (int i = 0; i < 82; i ++)  //печатаем график
	  {
		if (mass_p[i] > 0) 
			{
				if (mass_p[i] <= 15) 
					{
						myGLCD.drawLine(i + 1, 47, i + 1, 47 - mass_p[i]);
					}
				if (mass_p[i] > 15) 
					{
						myGLCD.drawLine(i + 1, 47, i + 1, 47 - 15);
					}
			}
		}
	}
myGLCD.update();
}
//-------------------------------------------------------------------------------------------------------
void lcd_menu()  //вывод на дисплей меню
{
myGLCD.clrScr();
myGLCD.setFont(TinyFontRus);
myGLCD.print(utf8rus("ПОРОГ 1"), 5, 0); myGLCD.printNumI(treviga_1, 55, 0); myGLCD.print("\xBC\xBD\xBE\xBF", RIGHT, 0);
myGLCD.print(utf8rus("ПОРОГ 2"), 5, 6); myGLCD.printNumI(treviga_2, 55, 6); myGLCD.print("\xBC\xBD\xBE\xBF", RIGHT, 6);
myGLCD.print(utf8rus("ПОДСВЕТКА"), 5, 12); 
if (podsvetka)  { myGLCD.print(utf8rus("ВКЛ."), RIGHT, 12); }
else  { myGLCD.print(utf8rus("ВЫКЛ."), RIGHT, 12);  }
myGLCD.print(utf8rus("ТИП. ГРАФИКА"), 5, 18);  myGLCD.printNumI(graph_type, 55, 18); myGLCD.print("0-1", RIGHT, 18); //usr
myGLCD.print(utf8rus("ОБН. ГРАФИКА"), 5, 24); myGLCD.printNumI(scrin_GRAF, 59, 24); myGLCD.print(utf8rus("СЕК."), RIGHT, 24);//
myGLCD.print(utf8rus("ИНДИКАЦИЯ"), 5, 30); //пункт меню выбора индикации частиц
switch (ind_ON)
	{
	case 0:
		myGLCD.print(utf8rus("ВЫКЛ."), RIGHT, 30); //индикация выключена
		break;	
	case 1:
		myGLCD.print(utf8rus("ЗВУК"), RIGHT, 30); //индикация звуком
		break; 	
	case 2:
		myGLCD.print(utf8rus("СВЕТ"), RIGHT, 30); //индикация светом
		break;	
	case 3:
		myGLCD.print(utf8rus("ЗВУК+СВЕТ"), RIGHT, 30); //индикация звуком и светом
		break; 
	case 4:
		myGLCD.print(utf8rus("ВИБРО"), RIGHT, 30); //индикация вибрацией
		break; 
		default:
		myGLCD.print("err", RIGHT, 30); //	если значение не равно 1,2,3 или 0 - выводим ошибку	
	}
myGLCD.print(utf8rus("ВЫХОД"), 5, 36);
myGLCD.print(utf8rus("СОХРАНИТЬ"), 5, 42);
myGLCD.print(">", 0, n_menu * 6);
myGLCD.update();
}
//----------------------------------------------------------------------------------------------------------------------
void lcd_sys()  //вывод на дисплей меню
{
VCC_read();
speed_nakachka ();//скорость накачки имлульсы/сек
myGLCD.clrScr();
myGLCD.setFont(TinyFontRus);
myGLCD.print(utf8rus("ОПОРН."), 5, 0); myGLCD.printNumF(opornoe, 2, CENTER, 0); myGLCD.print("VCC", 55, 0); myGLCD.printNumF(VCC, 2, RIGHT, 0);
hv_400 = hv_adc * opornoe * k_delitel / 255; //считем высокео перед выводом
myGLCD.print(utf8rus("НАКАЧКА"), 5, 6); myGLCD.printNumI(puls, 55, 6); myGLCD.printNumI(hv_400, RIGHT, 6);
myGLCD.print(utf8rus("СБРОС ДОЗЫ"), 5, 12); 
myGLCD.print(utf8rus("ВЫХОД"), 5, 18);
myGLCD.print(utf8rus("СОХРАНИТЬ"), 5, 24);
myGLCD.print(utf8rus("БЕТА"), 5, 30); myGLCD.printNumI(beta_time, 55, 30); myGLCD.print(utf8rus("МИН."), RIGHT, 30);
myGLCD.print(">", 0, sys_menu * 6);
myGLCD.print(utf8rus("СКОРОСТЬ"), 5, 40); myGLCD.printNumI(speed_nak, 40, 40); myGLCD.print(utf8rus("ИМП/СЕК"), RIGHT, 40);
myGLCD.update();
}
//---------------------------------------------------------------------------------------------------------------------
void zamer_beta() 
{// замер бета или продуктов
if (gotovo == 0) 
	{
		if (alarm_sound)  //если активен сигнал тревоги первого уровня
			{
				res_first_alarm(); //сбрасываем сигнал тревоги
			}
		myGLCD.clrScr();
		myGLCD.setFont(TinyFontRus);
		if (bet_z == 0)
			{
				myGLCD.drawBitmap(0, 0, beta_prev_1, 84, 48);
				/*
				myGLCD.print(utf8rus("РЕЖИМ РАЗНОСТНОГО"), CENTER, 0);
				myGLCD.print(utf8rus("ЗАМЕРА"), CENTER, 8); 
				myGLCD.drawLine(0, 16, 83, 16); 
				myGLCD.print(utf8rus("УСТАНОВИТЕ ПРИБОР"), CENTER, 20); 
				myGLCD.print(utf8rus("НА ПУСТУЮ КЮВЕТУ И"), CENTER, 28); 		
				*/
		
			}
		else if (bet_z == 1)
			{
				myGLCD.drawBitmap(0, 0, beta_prev_2, 84, 48);	
				/*
				myGLCD.print(utf8rus("ЗАМЕР ОБРАЗЦА"), CENTER, 0); 
				myGLCD.drawLine(0, 8, 83, 8); 		
				myGLCD.print(utf8rus("ЗАПОЛНИТЕ КЮВЕТУ"), CENTER, 12);
				myGLCD.print(utf8rus("ИЗМЕРЯЕМЫМ ВЕЩЕСТВОМ"), CENTER, 20);  
				myGLCD.print(utf8rus("УСТАНОВИТЕ ПРИБОР И"), CENTER, 28);
				*/	
			}

//  myGLCD.setFont(SmallFontRus);
//  myGLCD.print(utf8rus("Замер "), 20, 10); myGLCD.printNumI(bet_z, 55, 10);
myGLCD.setFont(SmallFontRus);
myGLCD.print(utf8rus("НАЖМИТЕ OK"), CENTER, 36);
myGLCD.update();
	}  
if (gotovo == 1) 
	{
		if (timer_seconds != beta_seconds) 
			{
				beta_seconds = timer_seconds;
				sek++;
				toch = !toch;
				if (sek >= 60) 
					{
						sek = 0;
						minute++;
					}
			}	
		byte otsup = 0;
		if (minute > 9) 
			{
				otsup = 5;
			}
		myGLCD.clrScr();
		battery();
    if  (bet_z < 2) //таймер выводим только пока идёт первый или второй замер
		{
			myGLCD.setFont(TinyFontRus);
			myGLCD.printNumI(minute, LEFT, 0);
			if (toch == 0) 
				{
					myGLCD.print(":", 5 + otsup, 0);
				}
			else 
				{
					myGLCD.print(" ", 5 + otsup, 0);
				} 
			myGLCD.printNumI(sek, 10 + otsup, 0); myGLCD.print("\xBC"":""\xB9", 23 + otsup, 0);
		}
    myGLCD.drawLine(0, 8, 83, 8);
    myGLCD.setFont(SmallFontRus);
    myGLCD.drawLine(40, 8, 40, 28);
    myGLCD.print(utf8rus("Замер0"), LEFT, 10); myGLCD.print(utf8rus("Замер1"), RIGHT, 10);
    myGLCD.printNumI(bet_z0, LEFT, 20); myGLCD.printNumI(bet_z1, RIGHT, 20);
    myGLCD.drawLine(0, 28, 83, 28);
    if (bet_z < 2) 
		{
			myGLCD.print(utf8rus("Идёт замер"), CENTER, 30); myGLCD.printNumI(bet_z, RIGHT, 30);
			myGLCD.printNumI(bet_r, CENTER, 38);
		}
    if (bet_z == 2) 
		{
			myGLCD.print(utf8rus("Результат"), CENTER, 30);
			myGLCD.printNumI(bet_r, CENTER, 38); myGLCD.print(utf8rus("мкР/ч"), RIGHT, 38);
		}
    myGLCD.update();
    if (bet_z == 0)  //первый замер
		{
			bet_z0 = bet_z0 + shet;
			shet = 0;
			if (minute >= beta_time) 
				{
					bet_z = 1;
					sek = 0;
					minute = 0;
					gotovo = 0; 
					tone (6,2000,70); //генерим писк 2000Гц 70миллисекунд на 6й ноге
				}
		}
    if (bet_z == 1)  //второй замер
	{
		bet_z1 = bet_z1 + shet;
		shet = 0;
		if (minute >= beta_time) 
			{
				bet_z = 2;
				sek = 0;
				minute = 0;
				tone (6,2000,70); //генерим писк 2000Гц 70миллисекунд на 6й ноге		
			}
    }
    if (bet_z == 2)  //результат
		{
			bet_r = bet_z1 - bet_z0;
//			bet_r = bet_r / (1.5 * beta_time);
			bet_r = bet_r / ((60.0/(float)geiger_counter_seconds) * (float)beta_time);
		}
	}
if (key_data == key_pressed_right)  //нажатие >>>
	{
		key_data = 0;  // обнуляем переменную функции кнопок для предотвращения ложных срабатываний далее по коду
		menu = 0;
		clear_poisk_variables();
	}
}
//-------------------------------------------------------------------------------------------------------------
void poisk_f() //режим поиска
{
int16_t shet_gr = 0;
if (poisk == 1) 
	{
		if (timer_seconds != count_and_dose_seconds) 
			{
				count_and_dose_seconds = timer_seconds;
					for (int i = 0; i < 254; i++)  //сдвигаем
					{
						mass_poisk[i] = mass_poisk[i + 1];
					}
				mass_poisk[254] = shet;
				if ((zam_poisk_counter < 254) && (zam_poisk_counter < geiger_counter_seconds))  //первый набор массива
					{
						fon_vr_poisk = fon_vr_poisk + shet;  						
						zam_poisk_counter++;
						fon = fon_vr_poisk*((float(geiger_counter_seconds))/(float(zam_poisk_counter))); 
//						fon_254 = fon;
						fon_254 = 0;
					}
				else if ((zam_poisk_counter < 254) && (zam_poisk_counter == geiger_counter_seconds))  //
					{		
						zam_poisk_counter++;
						fon_vr_poisk = fon_vr_poisk + shet; 
						fon = fon_vr_poisk*((float(geiger_counter_seconds))/(float(zam_poisk_counter)));
						fon_254 = fon;		
						fon_vr254 = fon_vr_poisk;
					}
				else if ((zam_poisk_counter < 254) && (zam_poisk_counter > geiger_counter_seconds))  //
					{
						fon_vr_poisk = 0;
						for (int i = zam_poisk_counter; i > 0; i--) 
							{
								fon_vr254 = fon_vr254 + mass_poisk[254-i];
							}
						for (int j = 254 - geiger_counter_seconds; j < 255; j++) 
							{	
								fon_vr_poisk = fon_vr_poisk + mass_poisk[j];
							}
						fon = fon_vr_poisk;
//						fon_254 = (float(fon_vr254))*((float(geiger_counter_seconds))/(float(zam_poisk_counter)));
						fon_254 = (float)fon_vr254*((float)geiger_counter_seconds/(float)zam_poisk_counter); 
						fon_vr254 = 0;
						zam_poisk_counter++;
					}	
				else if (zam_poisk_counter >= 254)  //набор массива
					{
						fon_vr_poisk = 0;
						fon_vr254 = 0;
						byte geiger_counter_seconds_reverse = 254 - geiger_counter_seconds;
						for (int i = 254; i > 0; i--) 
							{
								fon_vr254 = fon_vr254 + mass_poisk[i];
								if (i > geiger_counter_seconds_reverse)
									{
										fon_vr_poisk = fon_vr_poisk + mass_poisk[i];
									}
							}
						fon = fon_vr_poisk;
						fon_254 = (float(fon_vr254))*((float(geiger_counter_seconds))/254.0);
					}
				shet = 0;
				doz_v = doz_v + fon / 100.0 / 40.0;
				time_doza = time_doza + 1;
				if (doz_v - doza_vr >= save_DOZ)  //а не пора ли сохранить дозу ?
					{
						eeprom_wrD ();
						doza_vr = doz_v;
					}
//Serial.print(" zam_poisk_counter=");
//Serial.println(zam_poisk_counter);						
			}
		if (millis() - gr_milis >= scrin_GRAF * 1000) //счет для графика
			{
			gr_milis = millis();
			if (graph_type == 0)
				{
				val_ok = 0;//сброс удержания системного меню  
				for (uint8_t s = 254; s >= (255 - scrin_GRAF); s--) 
					{
						shet_gr = shet_gr + mass_poisk[s];
					}
				shet_gr = shet_gr / scrin_GRAF;
				for (int i = 0; i < 83; i++) // сдвигаем массив графика
					{
						mass_p[i] = mass_p[i + 1];
					}
				mass_p[82] = byte(shet_gr);
				if (GRAPH_count > 82) 
					{
						GRAPH_max = 5; 
						GRAPH_count = 0;
					}		
				for (int i = 0; i < 82; i++) 
					{
						if (mass_p[i] > GRAPH_max)
							{
								GRAPH_max = mass_p[i];
								GRAPH_count = 0;
							}
					}
				GRAPH_count++;	   
				}
			if (graph_type == 1)
				{
					val_ok = 0;//сброс удержания системного меню
					shet_gr = shet - shet_n;
					if (shet_gr < 0) 
						{
							shet_gr = 1;
						}
					shet_n = shet;
					for (int i = 0; i < 83; i++) 
						{
							mass_p[i] = mass_p[i + 1];
						}
					mass_p[82] = byte(shet_gr);
						
				
				}				
			}
	}
}
//----------------------------------------------------------------------------------------------------------------

void clear_poisk_variables ()
{
shet = 0;
fon = 0;
fon_254 = 0;
zam_poisk_counter = 0;
GRAPH_max = 5;
GRAPH_count = 0;
fon_vr254 = 0;
fon_vr_poisk = 0;
for (uint8_t i = 0; i < 83; i++) { mass_p[i] = 0; } // чистим массив графика
for (uint8_t i = 0; i < 254; i++) { mass_poisk[i] = 0; } // чистим массив поиска	
}

void signa ()  //индикация каждой частички звуком светом
{
shet_s = shet;  
if (!alarm_sound) //если флаг сигнала тревоги не поднят, генерим одиночные сигналы, озвучивающие пойманные частицы
    {
		if (!shet_s) {return;} //если залетели в функцию signa() при обнулении переменной shet_s - просто возвращаемся в точку вызова. Детальнее здесь: arduino.ru/forum/proekty/delaem-dozimetr?page=16#comment-318736
		switch (ind_ON) //проверяем, какой тип индикации выбран
			{
				case 0: //индикация выключена
					break;	
				case 1: //индикация звуком
					#ifdef buzzer_active //если задефайнен активный бузер
						{
							PORTD |= (1 << 6); //включаем бузер 
							delay(del_BUZZ); //длительность одиночного сигнала
							PORTD &= ~(1 << 6); //выключаем бузер 
						}
					#else //пассивный бузер
						{
							tone (6,1000,30); //генерим писк 1000Гц 30миллисекунд на 6й ноге
						}
					#endif 
					break; 	
				case 2: //индикация светом
					PORTB |= (1 << 5); //включаем светодиод
					delay(del_BUZZ); //длительность одиночного сигнала
					PORTB &= ~(1 << 5); //выключаем светодиод
					break;
	
				case 3: //индикация звуком и светом
					#ifdef buzzer_active //если задефайнен активный бузер
						{
							PORTB |= (1 << 5); //включаем светодиод
							PORTD |= (1 << 6); //включаем бузер 
							delay(del_BUZZ); //длительность одиночного сигнала
							PORTD &= ~(1 << 6); //выключаем бузер 
							PORTB &= ~(1 << 5); //выключаем светодиод
						}
					#else //пассивный бузер
						{
							PORTB |= (1 << 5); //включаем светодиод
							tone (6,1000,30); //генерим писк 1000Гц 30миллисекунд на 6й ноге
							delay(del_BUZZ);//длительность одиночного сигнала
							PORTB &= ~(1 << 5);//выключаем светодиод
						}
					#endif 
					break; 
				case 4: // индикация вибрацией
					PORTB |= (1 << 0); //включаем вибру
					delay(del_BUZZ); //длительность одиночного сигнала
					PORTB &= ~(1 << 0); // выключаем вибру			
			} 
    }
else // если активен сигнал тревоги, то только мигаем светодиодом (независимо от того, включён или нет светодиод в меню)
	{
	    PORTB |= (1 << 5); //включаем светодиод
		delay(del_BUZZ);
		PORTB &= ~(1 << 5);//выключаем светодиод
	}
}
//-------------------------------------------------------------------------------------------------
void Schet()  //прерывание от счетчика на пин 2
{
shet++;
}
//-------------------------------------------------------------------------------------------------
void generator() //накачка по обратной связи с АЦП
{
hv_adc  = Read_HV();
if (hv_adc < ADC_value)  //Значение АЦП при котором на выходе 400В
	{
		int c = puls;
		PORTD |= (1 << 5); //пин накачки
		while (c > 0) 
			{
				asm("nop");
				c--;
			}
		PORTD &= ~(1 << 5);//пин накачки
		speed_nakT++;
	}
}
//--------------------------------------------------------------------------------------------------
byte Read_HV () 
{
ADCSRA = 0b11100111;
#ifdef UNO_DIP //если при компилляции выбрана плата ArduinoUNO
ADMUX = 0b11100101;//выбор внутреннего опорного 1,1В и А5 
#else // если используется промини, нано или голый камень в tqfp
ADMUX = 0b11100110;//выбор внутреннего опорного 1,1В и А6
#endif  
for (int i = 0; i < 10; i++) 
	{
		while ((ADCSRA & 0x10) == 0);
		ADCSRA |= 0x10;
	}
result = 0;
for (int i = 0; i < 10; i++) 
	{
		while ((ADCSRA & 0x10) == 0);
		ADCSRA |= 0x10;
		result += ADCH;
	}
result /= 10;
return result;
}
//----------------------------------------------------------------------------------------------------
void battery()  //батарейка
{
if (bat_mill - millis() > 2000) 
	{
		bat_mill = millis();
		VCC_read();
	}
myGLCD.drawBitmap(59, 0, logo_bat, 24, 8);
myGLCD.setFont(TinyFontRus);
myGLCD.printNumF(VCC, 2, 65, 1);
}
//----------------------------------------------------------------------------------------------------
void VCC_read()  // Чтение напряжения батареи
{
ADCSRA = 0b11100111;
ADMUX = 0b01101110;//Выбор внешнего опорного+BG
_delay_ms(5);
while ((ADCSRA & 0x10) == 0);
ADCSRA |= 0x10;
byte resu = ADCH;
//ADCSRA &= ~(1 << ADEN);  // отключаем АЦП,
VCC = (opornoe * 255.0) / resu;
}
//----------------------------------------------------------------------------------------------------
void lcd_init() 
{
myGLCD.InitLCD();
myGLCD.setContrast(contrast);
myGLCD.clrScr();
myGLCD.drawBitmap(0, 0, logo_rag, 84, 48);
myGLCD.setFont(SmallFontRus);
//  myGLCD.print(utf8rus("Ардуино+"), CENTER, 32);
//  myGLCD.print(utf8rus("Дозиметр v1.07"), CENTER, 40);
myGLCD.update();
_delay_ms(1000);
}
//----------------------------------------------------------------------------------------------------
void eeprom_wrS ()  //запись настроек в память
{
  EEPROM.write(0, 222);
  EEPROM.write(1, treviga_1);
  EEPROM.write(2, podsvetka);
  EEPROM.write(3, graph_type);
  EEPROM.write(4, scrin_GRAF);
  EEPROM.write(5, ind_ON);
  EEPROM.write(6, puls);
  EEPROM.write(7, opornoe * 100);
  EEPROM.write(8, treviga_2);
  EEPROM.write(17, beta_time);
  myGLCD.clrScr();
  myGLCD.setFont(SmallFontRus);
  myGLCD.print(utf8rus("Сохранено"), CENTER, 24);
  myGLCD.update();
  _delay_ms(1000);
}
//-----------------------------------------------------------------------------------------------------
void eeprom_wrD ()  //запись настроек в память время накопления дозы
{
  EEPROM.put(9, time_doza);
  EEPROM.put(13, doz_v);   
}
//-----------------------------------------------------------------------------------------------------
void eeprom_readD ()  //чтение настроек из памяти время накопления дозы
{
  EEPROM.get(9, time_doza);
  EEPROM.get(13, doz_v);   
}
//-----------------------------------------------------------------------------------------------------
void eeprom_readS ()  //чтение настроек из памяти
{
  if (EEPROM.read(0) == 222) 
	{
		treviga_1 = EEPROM.read(1);
		podsvetka = EEPROM.read(2);
		graph_type = EEPROM.read(3);
		scrin_GRAF = EEPROM.read(4);
		ind_ON = EEPROM.read(5);
		puls = EEPROM.read(6);
		opornoe = EEPROM.read(7) / 100.0;
		treviga_2 = EEPROM.read(8);
		beta_time = EEPROM.read(17);
	}
_delay_ms(10);
}
//------------------------------------------------------------------------------------------------------
void nakachka() //первая накачка
{
byte n = 0;
while (n < 30) 
	{
		PORTD |= (1 << 5);//дергаем пин
		int c = puls;
		while (c > 0) 
			{
				asm("nop");
				c--;
			}
		PORTD &= ~(1 << 5);//дергаем пин
		n++;
		_delay_us(100);
	}
}
//------------------------------------------------------------------------------------------------------
void speed_nakachka ()  //скорость накачки имлульсы/сек
{
if (millis() - spNAK_milis >= 1000) 
	{
		spNAK_milis = millis();
		speed_nak = speed_nakT;
		speed_nakT = 0;
	}
}
//------------------------------------------------------------------------------------------------------
void time_d() 
{
	MONTH = time_doza / 2592000;
	DAY = (time_doza / 86400) % 30 ;
	HOUR = (time_doza / 3600) % 24 ;
	MIN = (time_doza / 60) % 60;
}
//------------------------------------------------------------------------------------------------------
void check_alarm_signal()  // устанавливаем сигнал непрерывной тревоги, если "tr" переключился в "1"
{
	if (!tr) // если счёт превысил аварийный порог, но флаг "tr" ещё не установлен
		{
			alarm_sound = 1; // поднимаем флаг аварийного сигнала
			alarm_milis = millis(); // запоминаем время начала тревоги
		}
}
//------------------------------------------------------------------------------------------------------
void res_first_alarm() //подпрограмма выключения тревоги (ручного или по истечении таймаута)
{
   alarm_sound = 0; // сбрасываем флаг звукового сигнала тревоги
   PORTB &= ~(1 << 0); // выключаем вибру
   periodical_alarm_variable = 0; // обнуляем переменную
   #ifdef buzzer_active //если задефайнен активный бузер
   PORTD &= ~(1 << 6); // выключаем бузер
   #else //пассивный бузер
   noTone(6);   //выключаем генерацию сигнала на 6й ноге
   #endif
}
//------------------------------------------------------------------------------------------------------
void ind_doze_time() //вывод времени накопления дозы на дисплей
{
	myGLCD.setFont(TinyFontRus);
	if (MONTH) // если есть месяцы
		{
			myGLCD.printNumI(MONTH, 0, 26);
			if(MONTH>99)
				{
					myGLCD.print("M", 13, 26);
				}
			else if (MONTH>9)
				{
					myGLCD.print("M", 9, 26);
				}
			else
				{
					myGLCD.print("M", 5, 26);
				}
		myGLCD.printNumI(DAY, 18, 26);
		if (DAY > 9) 
			{
				myGLCD.print("\xBB", 26, 26);
			}
		else
			{
				myGLCD.print("\xBB", 23, 26);
			}	
		}
	else if (DAY) // если нет месяцев, но есть дни
		{
			myGLCD.printNumI(DAY, 0, 26);
			if (DAY > 9) 
				{
					myGLCD.print("\xBB", 9, 26);
				}
			else
				{
					myGLCD.print("\xBB", 5, 26);
				}
			myGLCD.printNumI(HOUR, 18, 26);
			if (HOUR > 9) 
				{
					myGLCD.print("\xBA", 26, 26);
				}
			else 
				{
					myGLCD.print("\xBA", 23, 26);
				}
		}
	else // если нет дней
		{
			myGLCD.printNumI(HOUR, 0, 26);
			if (HOUR > 9) 
				{
					myGLCD.print("\xBA", 9, 26);
				}
			else
				{
					myGLCD.print("\xBA", 5, 26);
				}
			myGLCD.printNumI(MIN, 18, 26);
			if (MIN > 9) 
				{
					myGLCD.print("\xBC", 26, 26);
				}
			else
				{
					myGLCD.print("\xBC", 23, 26);
				}
		}		

}
//--------------------------------------------------------------------------------------------------------
byte get_key() // Функция определения нажатия и удержания кнопок
{
// версия 1 - для кратковременного нажатия значение возвращается при отпускании кнопки, для длительного - пока кнопка остаётся нажатой, с заданным интервалом
uint8_t trigger_push_hold_counter = 10; // задержка триггера кратковременного/длительного нажатия (проходов функции, умноженных на задержку "milliseconds_between_increment")  
uint8_t milliseconds_between_increment = 50; // интервал в миллисекундах между инкрементом счётчика нажатой кнопки 
static uint8_t val_kp, val_kl, val_ok;
static uint32_t key_delay_millis;
static uint32_t key_delay_after_hold_millis;
if ((millis() - key_delay_millis) > milliseconds_between_increment) //обрабатываем нажатия инкрементом переменной только если после предыдущей обработки прошло не менее "milliseconds_between_increment" миллисекунд
	{
	  if (!(PIND & (1 << PIND4)))  //нажатие >>>
		{
		val_kp++;  // инкрементируем счётчик
		if (val_kp > trigger_push_hold_counter) // если значение счётчика больше порога детектирования удержания клавиши 
			{
			 val_kp = 0; // сбрасываем счётчик 
			 key_delay_after_hold_millis = millis(); // запоминаем время
			 return key_holded_right; // возвращаем значение
			}
		}
	  if (!(PIND & (1 << PIND7)))  //нажатие <<<
		{
		val_kl++;  // инкрементируем счётчик
		if (val_kl > trigger_push_hold_counter) // если значение счётчика больше порога детектирования удержания клавиши
			{
				val_kl = 0; // сбрасываем счётчик
				key_delay_after_hold_millis = millis(); // запоминаем время 
				return key_holded_left; // возвращаем значение
			}   
		}
	  if (!(PIND & (1 << PIND3)))  //нажатие OK
		{
		val_ok++; // инкрементируем счётчик
		if (val_ok > trigger_push_hold_counter) // если значение счётчика больше порога детектирования удержания клавиши
			{
				val_ok = 0; // сбрасываем счётчик
				key_delay_after_hold_millis = millis(); // запоминаем время 
				return key_holded_ok; // возвращаем значение
			}       
		}
	  key_delay_millis = millis(); // запоминаем время 
	}
if (val_ok > 0) //если клавиша OK перед этим была нажата 
	{
		 if ((PIND & (1 << PIND3)) && ((millis() - key_delay_after_hold_millis) > (trigger_push_hold_counter * milliseconds_between_increment))) // если клавиша на данный момент отпущена и с момента последнего удержания любой клавиши прошёл интервал больше, чем один интервал удержания клавиши
			{
				val_ok = 0;  // сбрасываем счётчик
				return key_pressed_ok; // возвращаем значение
			}
	}
if (val_kp > 0) //если клавиша >>> перед этим была нажата 
	{
		if ((PIND & (1 << PIND4)) && ((millis() - key_delay_after_hold_millis) > (trigger_push_hold_counter * milliseconds_between_increment))) // если клавиша на данный момент отпущена и с момента последнего удержания любой клавиши прошёл интервал больше, чем один интервал удержания клавиши
			{
				val_kp = 0;  // сбрасываем счётчик 
				return key_pressed_right; // возвращаем значение
			}
	}
if (val_kl > 0) //если клавиша <<< перед этим была нажата 
	{
		if ((PIND & (1 << PIND7)) && ((millis() - key_delay_after_hold_millis) > (trigger_push_hold_counter * milliseconds_between_increment))) // если клавиша на данный момент отпущена и с момента последнего удержания любой клавиши прошёл интервал больше, чем один интервал удержания клавиши
			{
				val_kl = 0;  // сбрасываем счётчик  
				return key_pressed_left; // возвращаем значение
			}
	}
if (PIND & (1 << PIND4)) {val_kp = 0;} // если добрались до этой точки и кнопка не нажата - обнуляем счётчик (защита от появления "pressed" после "holded")
if (PIND & (1 << PIND7)) {val_kl = 0;} // если добрались до этой точки и кнопка не нажата - обнуляем счётчик (защита от появления "pressed" после "holded")
if (PIND & (1 << PIND3)) {val_ok = 0;} // если добрались до этой точки и кнопка не нажата - обнуляем счётчик (защита от появления "pressed" после "holded")
return 0; // если ни одна из кнопок не была нажата - возвращаем 0
}
//------------------------------------------------------------------------------------------------------------------------------
char *utf8rus(char *source) // функция преобразования utf8 для вывода кириллицы (by arduinec)
{
  int i,j,k;
  unsigned char n;
  char m[2] = { '0', '\0' };

  strcpy(target, ""); k = strlen(source); i = j = 0;

  while (i < k) {
    n = source[i]; i++;

    if (n >= 0xC0) {
      switch (n) {
        case 0xD0: {
          n = source[i]; i++;
          if (n == 0x81) { n = 0xA8; break; }
          if (n >= 0x90 && n <= 0xBF) n = n + 0x30;
          break;
        }
        case 0xD1: {
          n = source[i]; i++;
          if (n == 0x91) { n = 0xB8; break; }
          if (n >= 0x80 && n <= 0x8F) n = n + 0x70;
          break;
        }
      }
    }

    m[0] = n; strcat(target, m);
    j++; if (j >= maxString) break;
  }
  return target;
}

ISR (TIMER1_OVF_vect) // прерывание по таймеру, генерируемое каждую секунду
{ 
timer_seconds++ ; //инкремент переменной каждую секунду
if (timer_seconds > 59){timer_seconds = 0;}
}

// ________________ конец скетча, дальше можно не копировать _____________________




/*

ChangeLog by tekagi:

1.07.9_a		01.01.2019
  -переписан график (ранее график отобржал увеличение кол-ва импульсов над предыдущим временным интервалом, сейчас показывает усреднённое значение за выбранный интервал, масштабируя по максимальному значению за весь отображаемый на экране период)

1.07.8
  -добавлено управление вибросигналом из меню

1.07.7         19.06.2018
  -фикс учёта времени (переведено с миллис на таймер1, код взят из примера, выложенного dimax)
  -сигнал тревоги сделан прерывистым

1.07.6		  15.06.2018
  -начато добавление вибро

1.07.5        10.06.2018
  -мелкие изменения в графике: подкорректированы значки батарейки и аварийного сигнала (в батарейке значение напряжения было сдвинуто на пару пикселов, а колокольчик аварии перекрывался цифрами при повышении фона свыше 100мкР/ч);

1.07.4		  15.05.2018
  -исправлена лишняя секунда в режиме бета замера и немного изменена обработка тревоги первого уровня;

1.07.3		  28.04.2018
  -изменено содержание экранов между бета замерами. В промежуточной 1.07.2 сделал на стандартных функциях вывода библиотеки экрана, на двух экранах съело 12% оперативки. Пришлось переписать в виде картинки, попутно убрав вывод текста с начальной заставки и внеся его в картинку заставки.

1.07.1        25.04.2018
  -добавлен мелкий шрифт, русифицированы меню;

1.07          16.04.2018
  -начато добавление русского языка в интерфейсе. Спасибо kaktuc за русский шрифт к библиотеке и arduinec за функцию перекодирования выводимого на дисплей текста;
  -заменён дефайн "ADC" на "ADC_value", в новых версиях ArduinoIDE из-за этого возникала ошибка компилляции;

1.064         15.04.2018
  -добавлена возможность использования ArduinoUNO или голого камня atmega328p в DIP корпусе. Для переключения раскомментировать #define UNO в начале скетча, это переключит чтение высокого напряжения с делителя с пина A6 на A5;
  -добавлено переключение состояния подсветки при удержании ">>"
  
1.063.7       15.04.2018
  -попытка переписать обработку клавиш (вынесено в отдельную функцию);
  -выключен выход в системное меню из функций длительного и разностного замеров (оставлен только из основного режима "поиск");

1.063.6       25.03.2018
  -пофиксены кракозяблы при выводе "ANALIZ" в начале длительного замера;

1.063.5       14.01.18
  -пофиксена некорректная запись в еепром времени учёта дозы (писалось только 2 байта из четырёх);
  -добавлено преобразование микрорентген/час в миллирентгены/час в режиме поиска и длительного замера при фоне свыше 1000;

  
1.063.4       13.01.18
  -добавлена возможность включать индикацию светодиодом и бузером независимо друг от друга;

  
1.063.3 и ниже    12.11.2017
  -добавлена возможность выбрать активный или пассивный бузер;
  -пофиксен учёт фона во время нахождения в меню (при выходе из меню был скачок фона, поскольку в функции меню не было вызова poisk_f();   arduino.ru/forum/proekty/delaem-dozimetr?page=17#comment-320398   );
  -добавил режим непрерывной аварийной сигнализации при превышении первого порога, длительность сигнала настраивается в дефайне;
  -пофиксен серьёзный баг в режиме разностного замера (счёт импульсов начинался не с нажатия кнопки "ОК" при запуске второго цикла измерения, а сразу после окончания первого измерения, в результате разностный результат сильно завышался);
  -пофиксен паразитный сигнал при значении "shet = 0;"   arduino.ru/forum/proekty/delaem-dozimetr?page=16#comment-318736  ;

  P.S. Спасибо ImaSoft за подсказки и готовые кусочки кода.

*/
