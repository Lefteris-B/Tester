/* ************************************************************************
 *
 *   language specific global variables: English
 *
 *   (c) 2012-2016 by Markus Reschke
 *   based on code from Markus Frejek and Karl-Heinz K�bbeler
 *
 * ************************************************************************ */


/*
 *  English
 */

#if defined (UI_ENGLISH)

  /*
   *  constant strings (stored in EEPROM)
   */

  const unsigned char Running_str[] EEMEM = "Probing...";
  const unsigned char Weak_str[] EEMEM = "weak";
  const unsigned char Low_str[] EEMEM = "low";
  const unsigned char Timeout_str[] EEMEM = "Timeout";
  const unsigned char Failed1_str[] EEMEM = "No component";
  const unsigned char Failed2_str[] EEMEM = "found!";
  const unsigned char Done_str[] EEMEM = "done!";
  const unsigned char Select_str[] EEMEM = "Select";
  const unsigned char Selftest_str[] EEMEM = "Selftest";
  const unsigned char Adjustment_str[] EEMEM = "Adjustment";
  const unsigned char Save_str[] EEMEM = "Save";
  const unsigned char Load_str[] EEMEM = "Load";
  const unsigned char Show_str[] EEMEM = "Show Values";
  const unsigned char Remove_str[] EEMEM = "Remove";
  const unsigned char Create_str[] EEMEM = "Create";
  const unsigned char ShortCircuit_str[] EEMEM = "Short Circuit!";
  const unsigned char DischargeFailed_str[] EEMEM = "Battery?";
  const unsigned char Error_str[] EEMEM = "Error!";
  const unsigned char Exit_str[] EEMEM = "Exit";
  const unsigned char Checksum_str[] EEMEM = "Checksum";
  const unsigned char BJT_str[] EEMEM = "BJT";
  const unsigned char Thyristor_str[] EEMEM = "SCR";
  const unsigned char Triac_str[] EEMEM = "Triac";
  const unsigned char PUT_str[] EEMEM = "PUT";
  const unsigned char Bye_str[] EEMEM = "Bye!";

  #ifdef SW_SQUAREWAVE
    const unsigned char SquareWave_str[] EEMEM = "Square Wave";
  #endif

  #ifdef HW_ZENER
    const unsigned char Zener_str[] EEMEM = "Zener";
    const unsigned char Min_str[] EEMEM = "Min";
  #endif

  #ifdef HW_FREQ_COUNTER
    const unsigned char FreqCounter_str[] EEMEM = "Freq. Counter";
  #endif

  #ifdef SW_ENCODER
    const unsigned char Encoder_str[] EEMEM = "Rotary Encoder";
    const unsigned char TurnRight_str[] EEMEM = "Turn right!";
  #endif

  #ifdef SW_CONTRAST
    const unsigned char Contrast_str[] EEMEM = "Contrast";
  #endif

  #ifdef SW_IR_RECEIVER
    const unsigned char IR_Detector_str[] EEMEM = "IR detector";
  #endif

  #ifdef SW_OPTO_COUPLER
    const unsigned char OptoCoupler_str[] EEMEM = "Opto Coupler";
    const unsigned char Start_str[] EEMEM = "Start";
    const unsigned char None_str[] EEMEM = "None";
  #endif

  #ifdef SW_UJT
    const unsigned char UJT_str[] EEMEM = "UJT";
  #endif

#endif


/* ************************************************************************
 *   EOF
 * ************************************************************************ */
