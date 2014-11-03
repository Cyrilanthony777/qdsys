/* ********************************************************* */
/* Code to read Sonar LV-EZ0                                 */
/* We read this sensor using Pin Change Interrupt on PCINT20 */
/* ********************************************************* */

extern volatile unsigned long timer0_overflow_count;

ISR(PCINT2_vect) {

  if (sonar_status)
    {
    if ((PIND & B00010000)==0)  // Pulse end?
      {
      sonar_pulse_start_ms = sonar_start_ms;
      sonar_pulse_start_t0 = sonar_start_t0;
      sonar_pulse_end_ms = timer0_overflow_count;
      sonar_pulse_end_t0 = TCNT0;
      sonar_status = 0;
      sonar_new_data = 1;
      }
    }
  else if(PIND & B00010000)   // Pulse start
    {
    sonar_status = 1;
    sonar_start_ms = timer0_overflow_count;
    sonar_start_t0 = TCNT0;
    }
}

void Sonar_Init()
{
  PCMSK2 = _BV(PCINT20);  // Pin Change Mask (Only PCINT20)
  PCICR = _BV(PCIE2);     // Enable Pin Change interrupt PCINT2
}

int Get_Sonar_Pulse()
{
  int temp;
  
  temp = sonar_pulse_end_ms - sonar_pulse_start_ms;   // First calculate Timer0_overflows...
  temp = temp<<8;
  temp += (256-(int)sonar_pulse_start_t0) + (int)sonar_pulse_end_t0;  // Then Timer0 cycles (4us)
  temp = temp/16;  // Convert value (in 4us steps) to centimeters
  return (temp);
}

// This filter limits the max difference between readings and also aply an average filter
int Filter(int new_value, int old_value, int max_diff)
{
  int diff_values;
  int result;
  
  if (old_value==0)     // Filter is not initialized (no old value)
    return(new_value);
  diff_values = new_value - old_value;      // Difference with old reading
  if (diff_values>max_diff)   
    result = old_value+max_diff;    // We limit the max difference between readings
  else
    {
    if (diff_values<-max_diff)
      result = old_value-max_diff;        // We limit the max difference between readings
    else
      result = (new_value+old_value)>>1;  // Small filtering (average filter)
    }
  return(result); 
}

