// =============================================================================
//  SegwayRC v3.2  -  L. Vezzoni
//  Arduino Nano (ATmega328P @ 16 MHz) - MPU-6050 - 2x TB6600 - 2x NEMA17
//
// SegwayRC v1 firmware — L. Vezzoni
// Licensed under CC BY-NC-SA 4.0, same as the 3D model.
// Balancing algorithm derived from Joop Brokking's YABR project.
//
//  NOVITA' RISPETTO ALLA v3.1
//  --------------------------
//  1) Generatore di passi DDS (accumulatore di fase) al posto del divisore
//     intero: la frequenza e' regolabile con risoluzione di 0,76 Hz invece
//     che a scalini del 20% alle alte velocita'. Tetto alzato da ~12,4 kHz
//     a 25 kHz per motore.
//  2) BUG FISSATO: il fondo scala del giroscopio era impostato a +/-250 dps
//     ma il fattore di integrazione (0,000031) corrispondeva a +/-500 dps.
//     L'angolo integrato correva al doppio della velocita' reale. Ora il
//     registro e' su +/-500 dps, coerente col fattore (e con piu' margine
//     prima della saturazione durante una caduta veloce).
//  3) BUG FISSATO: PCICR abilitava solo il gruppo PCIE0 (pin 8,9) ma PCMSK2
//     era programmato per il pin 7. Il canale 1 (mode) veniva campionato
//     solo per caso. Ora e' abilitato anche PCIE2.
//  4) BUG FISSATO: "Beccheggio_gyro -= Calibrazione_imbardata" corrompeva il
//     valore di beccheggio con l'offset dell'imbardata. Rimosso.
//  5) La print() seriale su ogni ciclo mandava ~25 byte ogni 2 ms = 2,2 ms di
//     trasmissione a 115200. Saturava il buffer e BLOCCAVA il loop, rompendo
//     il tempo di ciclo. Ora e' limitata e non bloccante.
//  6) Rimosso il codice morto nella gestione del throttle (i due if con
//     +/-0,005 si annullavano a vicenda e il setpoint veniva comunque
//     riscritto al ciclo successivo).
//  7) Aggiunto failsafe radiocomando (se il ricevitore tace -> stick a zero).
//  8) Aggiunta rampa di accelerazione per non perdere passi.
//  9) Tutti i numeri magici sono ora #define commentati in cima.
//
//  ATTENZIONE ALLA TARATURA
//  ------------------------
//  Se MICROPASSI o FREQ_MAX_HZ cambiano, la relazione fra uscita del PID e
//  velocita' reale cambia: RITARA Kp/Ki/Kd partendo piu' bassi.
//  Prima accensione: robot SOLLEVATO da terra, ruote libere.
// =============================================================================

#include <Wire.h>
#include <util/atomic.h>

// ---------------------------------------------------------------- MECCANICA -
#define MICROPASSI          8        // <<< DEVE corrispondere ai DIP dei TB6600
#define PASSI_GIRO          200      // passi/giro del motore (1,8 gradi)
#define DIAM_RUOTA_M        0.105f   // diametro ruota in metri

// Frequenza di passo massima per motore [Hz]. Vincoli:
//   a) l'ISR gira a 50 kHz e puo' emettere al massimo 1 impulso ogni 2 tick
//      -> tetto assoluto 25000 Hz
//   b) i TB6600 hanno ingressi optoisolati: sopra ~20 kHz iniziano a perdere
//      impulsi. Con driver digitali (DM542) puoi salire.
//   c) se il motore stalla o "canta" senza girare, sei oltre la sua coppia:
//      abbassa questo valore oppure alza la tensione di alimentazione.
// Con 12500 Hz e 1/8 di passo -> 2,56 m/s teorici (9,2 km/h).
#define FREQ_MAX_HZ         12500.0f

// -------------------------------------------------------------- REGOLATORE --
float Equilibrio   = -5.0f;   // [gradi] offset di montaggio dell'accelerometro
float Kp           = 21.0f;   // guadagno proporzionale
float Ki           = 1.0f;    // guadagno integrale
float Kd           = 0.01f;   // guadagno derivativo

#define PID_MAX             600.0f   // saturazione uscita PID
#define PID_I_MAX           600.0f   // saturazione termine integrale (anti-windup)
#define BANDA_MORTA_PID     8.0f     // sotto questo valore i motori si fermano
#define ANGOLO_CADUTA       30.0f    // [gradi] oltre i quali il robot si arrende

// Retroazione di velocita': somma l'uscita del PID all'errore d'angolo.
// E' quello che fa "assestare" il robot a una velocita' costante invece di
// accelerare all'infinito. Piu' basso = il robot accelera di piu' per lo
// stesso angolo di inclinazione, ma diventa oscillatorio. Non scendere
// sotto ~0,008 senza ritarare.
#define K_RETRO_VELOCITA    0.015f

#define ANGOLO_MAX_COMANDO  5.0f    // [gradi] inclinazione max chiesta dallo stick
#define GUADAGNO_STERZO     1.0f     // moltiplica il differenziale di sterzo
#define BANDA_MORTA_RC      5        // zona morta stick (su scala -100..+100)

// Limite di variazione della frequenza di passo per ciclo (2 ms).
// Serve a non perdere passi: un passo-passo non puo' saltare istantaneamente
// da 0 a 12 kHz. Troppo basso = il robot non riesce a raddrizzarsi e cade.
// 1500 Hz/ciclo = 750 kHz/s, molto permissivo. Abbassa solo se perdi passi.
#define RAMPA_MAX_HZ_CICLO  1500.0f

// ------------------------------------------------------------------ SENSORE -
#define MPU_INDIRIZZO       0x68
#define DT_S                0.002f   // periodo di ciclo [s] = 500 Hz
#define GYRO_LSB_PER_DPS    65.5f    // fondo scala +/-500 dps
#define ACC_LSB_1G          8192.0f  // fondo scala +/-4 g
const float FATT_GYRO = DT_S / GYRO_LSB_PER_DPS;   // ~0,00003053

#define COMPL_A             0.9990f  // peso del giroscopio nel filtro complementare
#define COMPL_B             0.0010f  // peso dell'accelerometro (A + B = 1)

// ---------------------------------------------------------------- DEBUG/RC ---
#define DEBUG_SERIALE       1        // 0 per disattivarlo del tutto
#define DEBUG_OGNI_N_CICLI  100      // 100 cicli x 2 ms = stampa ogni 200 ms
#define RC_TIMEOUT_US       500000UL // 0,5 s senza impulsi -> failsafe

// ------------------------------------------------------------- MASCHERE PIN --
// D2 = PUL sinistro, D3 = DIR sinistro, D4 = PUL destro, D5 = DIR destro
#define M_PUL_SX  0b00000100
#define M_DIR_SX  0b00001000
#define M_PUL_DX  0b00010000
#define M_DIR_DX  0b00100000
#define LED_PIN   13

// =============================================================================
//  VARIABILI
// =============================================================================

// Condivise con l'ISR del generatore di passi (accesso atomico obbligatorio)
volatile uint16_t inc_sx = 0, inc_dx = 0;   // incrementi DDS
volatile uint8_t  dir_sx = 0, dir_dx = 0;   // 0 = avanti, 1 = indietro

// Condivise con l'ISR del radiocomando
volatile unsigned long timer_rc[4];
volatile byte  ultimo_canale[3];
volatile int   input_rc[3];
volatile unsigned long ultimo_impulso_rc = 0;

// Regolatore
float PID_errore, PID_i_mem, PID_setpoint, Uscita_PID, PID_ultimo_errore;
float PID_uscita_sx, PID_uscita_dx;
float PID_setpoint_bilanciamento;
float Angolo_gyro, Angolo_acc;
float freq_sx_prec = 0, freq_dx_prec = 0;

// Sensore
int  Beccheggio_gyro, Imbardata_gyro, Accelerometro_z;
long Calibrazione_imbardata, Calibrazione_beccheggio;

// Servizio
byte Avvio = 0;
int  throttle, steering, mode;
unsigned long Tempo_ciclo;
uint16_t contatore_debug = 0;

// =============================================================================
//  SETUP
// =============================================================================
void setup(){
  Serial.begin(115200);
  Wire.begin();
  TWBR = 12;                     // I2C a 400 kHz

  pinMode(2, OUTPUT); pinMode(3, OUTPUT);
  pinMode(4, OUTPUT); pinMode(5, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  PORTD &= ~(M_PUL_SX | M_PUL_DX);

  // --- Timer2: interrupt ogni 20 us (50 kHz) per il generatore di passi ---
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);         // prescaler 8 -> 2 MHz
  OCR2A   = 39;                  // 40 conteggi = 20 us
  TCCR2A |= (1 << WGM21);        // modalita' CTC

  // --- MPU-6050 ---
  scriviMPU(0x6B, 0x00);   // PWR_MGMT_1: sveglia il sensore
  scriviMPU(0x1B, 0x08);   // GYRO_CONFIG: +/-500 dps  <-- CORRETTO (era 0x00)
  scriviMPU(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g
  scriviMPU(0x1A, 0x03);   // CONFIG: DLPF ~43 Hz
                           // (prova 0x02 = 94 Hz se vuoi meno ritardo di fase,
                           //  a costo di piu' rumore)

  // --- Interrupt radiocomando ---
  // v3.1 abilitava solo PCIE0: il pin 7 (PCINT23, gruppo 2) non generava mai
  // interrupt. Ora entrambi i gruppi sono attivi.
  PCICR  |= (1 << PCIE0) | (1 << PCIE2);
  PCMSK0 |= (1 << PCINT0);   // D8  - throttle
  PCMSK0 |= (1 << PCINT1);   // D9  - steering
  PCMSK2 |= (1 << PCINT23);  // D7  - mode

  // --- Calibrazione offset giroscopio (il robot deve stare FERMO) ---
  for(int i = 0; i < 800; i++){
    if(i % 100 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Wire.beginTransmission(MPU_INDIRIZZO);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU_INDIRIZZO, 4);
    Calibrazione_imbardata  += Wire.read() << 8 | Wire.read();
    Calibrazione_beccheggio += Wire.read() << 8 | Wire.read();
    delayMicroseconds(1700);
  }
  Calibrazione_imbardata  /= 800;
  Calibrazione_beccheggio /= 800;
  digitalWrite(LED_PIN, LOW);

#if DEBUG_SERIALE
  Serial.println(F("\nSegwayRC v3.2"));
  Serial.print(F("Micropassi 1/"));   Serial.println(MICROPASSI);
  Serial.print(F("Freq max Hz "));    Serial.println(FREQ_MAX_HZ);
  Serial.print(F("Vel max m/s "));
  Serial.println(FREQ_MAX_HZ / (PASSI_GIRO * MICROPASSI) * 3.14159f * DIAM_RUOTA_M);
#endif

  Tempo_ciclo = micros() + (unsigned long)(DT_S * 1000000.0f);
}

// =============================================================================
//  LOOP PRINCIPALE  (500 Hz)
// =============================================================================
void loop(){

  // ---------------------------------------------------- lettura radiocomando -
  int rc[3];
  unsigned long t_rc;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){       // gli int sono a 16 bit: lettura
    rc[0] = input_rc[0];                   // non atomica -> valori corrotti
    rc[1] = input_rc[1];
    rc[2] = input_rc[2];
    t_rc  = ultimo_impulso_rc;
  }

  if((micros() - t_rc) > RC_TIMEOUT_US){   // FAILSAFE: ricevitore muto
    throttle = 0;
    steering = 0;
    mode     = 0;
  } else {
    mode     = rc[0];
    throttle = map(constrain(rc[1], 1000, 2000), 1000, 2000, -100, 100);
    steering = map(constrain(rc[2], 1000, 2000), 1000, 2000, -100, 100);
  }

  // -------------------------------------------------------- stima dell'angolo -
  // Accelerometro: da' l'angolo assoluto ma e' rumoroso e sensibile alle
  // accelerazioni del veicolo.
  Wire.beginTransmission(MPU_INDIRIZZO);
  Wire.write(0x3F);                        // ACCEL_ZOUT_H
  Wire.endTransmission();
  Wire.requestFrom(MPU_INDIRIZZO, 2);
  Accelerometro_z = Wire.read() << 8 | Wire.read();
  Accelerometro_z = constrain(Accelerometro_z, -(int)ACC_LSB_1G, (int)ACC_LSB_1G);
  Angolo_acc = asin((float)Accelerometro_z / ACC_LSB_1G) * 57.296f - Equilibrio;

  // Avvio automatico quando il robot viene messo in verticale
  if(Avvio == 0 && Angolo_acc > -0.5f && Angolo_acc < 0.5f){
    Angolo_gyro = Angolo_acc;
    Avvio = 1;
  }

  // Giroscopio: preciso nel breve periodo ma deriva. Si integra.
  Wire.beginTransmission(MPU_INDIRIZZO);
  Wire.write(0x43);                        // GYRO_XOUT_H
  Wire.endTransmission();
  Wire.requestFrom(MPU_INDIRIZZO, 4);
  Imbardata_gyro  = Wire.read() << 8 | Wire.read();
  Beccheggio_gyro = Wire.read() << 8 | Wire.read();
  Beccheggio_gyro -= Calibrazione_beccheggio;
  Angolo_gyro += Beccheggio_gyro * FATT_GYRO;

  // Filtro complementare: il giroscopio domina, l'accelerometro corregge
  // lentamente la deriva. Costante di tempo = DT * A / B = 2 s.
  Angolo_gyro = Angolo_gyro * COMPL_A + Angolo_acc * COMPL_B;

  // ------------------------------------------------------------------- PID --
  PID_errore = Angolo_gyro - PID_setpoint_bilanciamento - PID_setpoint;

  // Retroazione di velocita': "conta" anche quanto stanno gia' correndo le
  // ruote, cosi' il robot si stabilizza a velocita' costante.
  if(Uscita_PID > 10.0f || Uscita_PID < -10.0f)
    PID_errore += Uscita_PID * K_RETRO_VELOCITA;

  PID_i_mem += Ki * PID_errore;
  PID_i_mem  = constrain(PID_i_mem, -PID_I_MAX, PID_I_MAX);

  Uscita_PID = Kp * PID_errore + PID_i_mem + Kd * (PID_errore - PID_ultimo_errore);
  Uscita_PID = constrain(Uscita_PID, -PID_MAX, PID_MAX);
  PID_ultimo_errore = PID_errore;

  if(Uscita_PID < BANDA_MORTA_PID && Uscita_PID > -BANDA_MORTA_PID)
    Uscita_PID = 0;

  // Caduta o non ancora avviato -> tutto a zero
  if(Angolo_gyro > ANGOLO_CADUTA || Angolo_gyro < -ANGOLO_CADUTA || Avvio == 0){
    Uscita_PID = 0;
    PID_i_mem  = 0;
    Avvio      = 0;
    PID_setpoint_bilanciamento = 0;
  }

  // --------------------------------------------------- comandi di movimento --
  PID_uscita_sx = Uscita_PID;
  PID_uscita_dx = Uscita_PID;

  // Sterzo: differenziale fra le due ruote
  if(steering > BANDA_MORTA_RC || steering < -BANDA_MORTA_RC){
    PID_uscita_sx += steering * GUADAGNO_STERZO;
    PID_uscita_dx -= steering * GUADAGNO_STERZO;
  }

  // Avanti/indietro: lo stick chiede un ANGOLO di inclinazione, non una
  // velocita'. E' il modo giusto: un segway accelera solo se si inclina.
  if(throttle > BANDA_MORTA_RC || throttle < -BANDA_MORTA_RC){
    PID_setpoint = ANGOLO_MAX_COMANDO * throttle / 100.0f;
  } else {
    // Rientro morbido a zero quando si rilascia lo stick
    if(PID_setpoint > 0.1f)       PID_setpoint -= 0.05f;
    else if(PID_setpoint < -0.1f) PID_setpoint += 0.05f;
    else                          PID_setpoint  = 0.0f;
  }

  // Auto-ricerca del punto di equilibrio: se da fermo il robot deve comunque
  // spingere per stare su, significa che il baricentro non e' sopra l'asse.
  if(PID_setpoint == 0.0f){
    if(Uscita_PID < 0) PID_setpoint_bilanciamento += 0.0015f;
    if(Uscita_PID > 0) PID_setpoint_bilanciamento -= 0.0015f;
  }

  // ------------------------------------------- conversione in passi/secondo --
  float f_sx = frequenzaDaPID(PID_uscita_sx);
  float f_dx = frequenzaDaPID(PID_uscita_dx);

  f_sx = limitaRampa(f_sx, freq_sx_prec);  freq_sx_prec = f_sx;
  f_dx = limitaRampa(f_dx, freq_dx_prec);  freq_dx_prec = f_dx;

  // DDS: incremento = freq / 50000 * 65536
  uint16_t i_sx = (uint16_t)(fabs(f_sx) * 1.31072f);
  uint16_t i_dx = (uint16_t)(fabs(f_dx) * 1.31072f);
  uint8_t  d_sx = (f_sx < 0) ? 1 : 0;
  uint8_t  d_dx = (f_dx < 0) ? 1 : 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    inc_sx = i_sx;  dir_sx = d_sx;
    inc_dx = i_dx;  dir_dx = d_dx;
  }

  // ----------------------------------------------------------------- debug ---
#if DEBUG_SERIALE
  if(++contatore_debug >= DEBUG_OGNI_N_CICLI){
    contatore_debug = 0;
    // Non bloccante: stampa solo se c'e' spazio nel buffer di trasmissione
    if(Serial.availableForWrite() > 40){
      Serial.print(Angolo_gyro, 1); Serial.print(' ');
      Serial.print(PID_setpoint, 1); Serial.print(' ');
      Serial.print((int)Uscita_PID); Serial.print(' ');
      Serial.println((int)f_sx);
    }
  }
#endif

  // ------------------------------------------- tempo di ciclo fisso a 2 ms ---
  // Se questo while non attende mai, il loop e' in ritardo: la stima
  // dell'angolo diventa sbagliata perche' FATT_GYRO presuppone DT_S esatto.
  while(Tempo_ciclo > micros());
  Tempo_ciclo += (unsigned long)(DT_S * 1000000.0f);
}

// =============================================================================
//  FUNZIONI DI SUPPORTO
// =============================================================================

void scriviMPU(byte reg, byte val){
  Wire.beginTransmission(MPU_INDIRIZZO);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Mappa l'uscita del PID (+/-PID_MAX) sulla frequenza di passo (+/-FREQ_MAX_HZ).
// La curva iperbolica e' quella originale di YABR: compensa il fatto che la
// coppia di un passo-passo cala con la velocita', quindi serve piu' "comando"
// per ottenere l'ultimo tratto di giri. E' normalizzata perche' a PID_MAX
// restituisca esattamente FREQ_MAX_HZ.
// Metti MAPPA_LINEARE a 1 se preferisci una risposta lineare (piu' facile da
// modellare, ma dovrai rialzare Kp).
#define MAPPA_LINEARE 0

float frequenzaDaPID(float x){
  float a = fabs(x);
  if(a < 1.0f) return 0.0f;
  if(a > PID_MAX) a = PID_MAX;

  float f;
#if MAPPA_LINEARE
  f = FREQ_MAX_HZ * a / PID_MAX;
#else
  const float k_max = (PID_MAX + 9.0f) / (1091.0f - PID_MAX);
  float k = (a + 9.0f) / (1091.0f - a);
  f = FREQ_MAX_HZ * k / k_max;
#endif

  if(f > 25000.0f) f = 25000.0f;      // tetto fisico dell'ISR
  return (x < 0) ? -f : f;
}

// Limita la variazione di frequenza per ciclo: un motore passo-passo che
// riceve un salto di frequenza troppo grande perde il sincronismo e si ferma
// (e il robot cade). Questo e' il motivo per cui i passo-passo hanno bisogno
// di rampe mentre un motore CC no.
float limitaRampa(float richiesta, float precedente){
  float delta = richiesta - precedente;
  if(delta >  RAMPA_MAX_HZ_CICLO) return precedente + RAMPA_MAX_HZ_CICLO;
  if(delta < -RAMPA_MAX_HZ_CICLO) return precedente - RAMPA_MAX_HZ_CICLO;
  return richiesta;
}

// =============================================================================
//  ISR GENERATORE DI PASSI  (50 kHz)
// =============================================================================
//  Sintesi digitale diretta (DDS). Un accumulatore a 16 bit viene incrementato
//  a ogni tick; ogni volta che va in overflow si emette un impulso.
//    frequenza = incremento / 65536 * 50000 Hz
//  Risoluzione: 0,76 Hz su tutto il campo. Il metodo precedente (divisore
//  intero di tick) aveva scalini del 20% oltre i 10 kHz, cioe' il controllo
//  non riusciva piu' a dosare la velocita' proprio dove serviva.
// =============================================================================
ISR(TIMER2_COMPA_vect){
  static uint16_t acc_sx = 0, acc_dx = 0;
  static uint8_t  dir_sx_att = 255, dir_dx_att = 255;
  uint16_t prec;

  // Chiude gli impulsi aperti al tick precedente (larghezza 20 us, ampiamente
  // sopra il minimo richiesto dai TB6600)
  PORTD &= ~(M_PUL_SX | M_PUL_DX);

  // ---- motore sinistro ----
  if(dir_sx_att != dir_sx){
    dir_sx_att = dir_sx;
    if(dir_sx_att) PORTD &= ~M_DIR_SX; else PORTD |= M_DIR_SX;
    acc_sx = 0;              // salta un tick: garantisce >=20 us di setup
  } else {                   // fra il cambio di DIR e il fronte di PUL
    prec = acc_sx;
    acc_sx += inc_sx;
    if(acc_sx < prec) PORTD |= M_PUL_SX;
  }

  // ---- motore destro (DIR invertita: montaggio speculare) ----
  if(dir_dx_att != dir_dx){
    dir_dx_att = dir_dx;
    if(dir_dx_att) PORTD |= M_DIR_DX; else PORTD &= ~M_DIR_DX;
    acc_dx = 0;
  } else {
    prec = acc_dx;
    acc_dx += inc_dx;
    if(acc_dx < prec) PORTD |= M_PUL_DX;
  }
}

// =============================================================================
//  ISR RADIOCOMANDO
// =============================================================================
ISR(PCINT0_vect){
  unsigned long ora = micros();

  // Canale 1 - mode (D7, PIND bit 7)
  if(ultimo_canale[0] == 0 && (PIND & B10000000)){
    ultimo_canale[0] = 1;  timer_rc[1] = ora;
  } else if(ultimo_canale[0] == 1 && !(PIND & B10000000)){
    ultimo_canale[0] = 0;  input_rc[0] = ora - timer_rc[1];
    ultimo_impulso_rc = ora;
  }

  // Canale 2 - throttle (D8, PINB bit 0)
  if(ultimo_canale[1] == 0 && (PINB & B00000001)){
    ultimo_canale[1] = 1;  timer_rc[2] = ora;
  } else if(ultimo_canale[1] == 1 && !(PINB & B00000001)){
    ultimo_canale[1] = 0;  input_rc[1] = ora - timer_rc[2];
    ultimo_impulso_rc = ora;
  }

  // Canale 3 - steering (D9, PINB bit 1)
  if(ultimo_canale[2] == 0 && (PINB & B00000010)){
    ultimo_canale[2] = 1;  timer_rc[3] = ora;
  } else if(ultimo_canale[2] == 1 && !(PINB & B00000010)){
    ultimo_canale[2] = 0;  input_rc[2] = ora - timer_rc[3];
    ultimo_impulso_rc = ora;
  }
}

// Il pin 7 appartiene al gruppo PCINT2: stessa routine.
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
