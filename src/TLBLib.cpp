#include "TLBLib.h"

TLBLib::TLBLib(uint8_t ENA_pin, sendFunction_type sendFunction, beginFunction_type beginFunction, endFunction_type endFunction) :
  _ENA_pin       (ENA_pin),
  _sendFunction  (sendFunction),
  _beginFunction (beginFunction),
  _endFunction   (endFunction)
{}

void TLBLib::errorFunction(errorFunction_type function)
{
  //Save the provided function callback.
  _errorFunction = function;
}

void TLBLib::begin()
{
  //Call the user function if it was defined.
  if (_beginFunction)
  {
    _beginFunction();
  }
  
  //Configure the ENA line.
  pinMode(_ENA_pin, INPUT);
}

void TLBLib::end()
{
  //Call the user function if it was defined.
  if (_endFunction)
  {
    _endFunction();
  }
  
  //Reset the message rate timer.
  message_rate_timer = 0;
}

void TLBLib::update()
{
  //Maintain the connection.
  keep_alive();
  
  //Check for errors.
  check_error_pulse();
}

void TLBLib::turnOff()
{
  //Ensure a distance of at least MIN_MESSAGE_RATE_ms between the last message and the "turn off".
  unsigned long delay_already_done = millis() - message_rate_timer;
  if (delay_already_done < MIN_MESSAGE_RATE_ms)
  {
    delay(MIN_MESSAGE_RATE_ms - delay_already_done);
  }
  
  //The message will be sent until a confirmation is received.
  uint8_t attempt_counter = 0;
  while (true)
  {
    //Send the "turn off" message.
    send_opcode(TURN_OFF_OPCODE);
    
    //Stop trying after 10 attempts.
    if (attempt_counter++ >= 10)
    {
      return;
    }
    
    //After sending, wait to see if the message was accepted.
    unsigned long pulse_timer = millis();
    while (millis() - pulse_timer < RPT_TIMEOUT_ms)
    {
      //Check if ENA is HIGH.
      if (digitalRead(_ENA_pin))
      {
        //Wait for ENA to go LOW.
        while (digitalRead(_ENA_pin));
        
        //Reset the timer.
        message_rate_timer = millis();
        
        //Ensure the next message will be distanced correctly.
        delay(MIN_MESSAGE_RATE_ms);
        
        //Exit.
        return;
      }
    }
    
    //After an unsuccessful attempt, delay more if necessary to ensure MIN_MESSAGE_RATE_ms.
    if (MIN_MESSAGE_RATE_ms > RPT_TIMEOUT_ms)
    {
      delay(MIN_MESSAGE_RATE_ms - RPT_TIMEOUT_ms);
    }
  }
}

TLBLib::sendError TLBLib::send(uint8_t* data)
{
  //Send the opcode (the first byte of the buffer).
  uint8_t opcode = data[0];
  if (!send_opcode(opcode))
  {
    return exit_send(FAIL);
  }
  
  //Wait for the rest of the message to be requested.
  if (!wait_ENA(HIGH, REQ_TIMEOUT_us))
  {
    return exit_send(FAIL);
  }
  
  //The length is the second byte of the buffer, incremented by 1 to add the
  //CRC byte at the end.
  uint8_t data_length = data[1] + 1;
  
  //The CRC calculation starts from the opcode byte.
  uint8_t crc = opcode;
  
  //Send each byte.
  for (uint8_t i = 0; i < data_length; i++)
  {
    //First send the length byte.
    if (i == 0)
    {
      send_byte(data_length, &crc);
    }
    //Then send the rest of the bytes.
    else
    {
      send_byte(data[i + 1], &crc);
    }
    //The CRC is updated automatically by send_byte().
    
    //Wait for the byte to be acknowledged.
    if (!wait_ENA(LOW, ACK_TIMEOUT_us))
    {
      return exit_send(FAIL);
    }
  
    //Wait for the next byte to be requested.
    if (!wait_ENA(HIGH, REQ_TIMEOUT_us))
    {
      return exit_send(FAIL);
    }
  }
  
  //Send the CRC byte.
  send_byte(crc - 1);
  
  //Wait for the CRC byte to be acknowledged.
  if (!wait_ENA(LOW, ACK_TIMEOUT_us))
  {
    return exit_send(FAIL);
  }
  
  //After sending, wait to see if the message must be repeated.
  unsigned long repeat_pulse_timer = millis();
  while (millis() - repeat_pulse_timer < RPT_TIMEOUT_ms)
  {
    //Check if ENA is HIGH.
    if (digitalRead(_ENA_pin))
    {
      //Wait for ENA to go LOW.
      while (digitalRead(_ENA_pin));
      return exit_send(REPEAT);
    }
  }
  
  //Finished sending.
  return exit_send(SUCCESS);
}

void TLBLib::keep_alive()
{
  //Check if the timer is up.
  if (millis() - keep_alive_timer >= KEEP_ALIVE_RATE_ms) {
    //Ensure a distance of at least MIN_MESSAGE_RATE_ms between the last message and the "keep alive".
    unsigned long delay_already_done = millis() - message_rate_timer;
    if (delay_already_done < MIN_MESSAGE_RATE_ms)
    {
      delay(MIN_MESSAGE_RATE_ms - delay_already_done);
    }
    
    //Send the "keep alive" message.
    send_opcode(KEEP_ALIVE_OPCODE);
    
    //Reset the timers.
    keep_alive_timer = millis();
    message_rate_timer = millis();
  }
}

void TLBLib::check_error_pulse()
{
  //Check if ENA is HIGH.
  if (digitalRead(_ENA_pin))
  {
    //Wait for ENA to go LOW and measure the duration of the pulse.
    unsigned long pulse_start = millis();
    while (digitalRead(_ENA_pin));
    unsigned long pulse_end = millis();
    
    //If an _errorFunction is defined, call it with the pulse's duration.
    if (_errorFunction)
    {
      _errorFunction(pulse_end - pulse_start);
    }
  }
}

bool TLBLib::send_opcode(uint8_t opcode)
{
  //Ensure the line is free.
  if (!wait_ENA(LOW, ACK_TIMEOUT_us))
  {
    return false;
  }
  
  //Send the opcode.
  start_ENA();
  send_byte(opcode);
  stop_ENA();
  
  //Wait for the opcode to be acknowledged.
  if (!wait_ENA(LOW, ACK_TIMEOUT_us))
  {
    return false;
  }
  
  //Sent successfully.
  return true;
}

void TLBLib::send_byte(uint8_t data, uint8_t* crc)
{
  //If a CRC byte was provided, update it with the current data.
  if (crc)
  {
    *crc ^= data;
  }
  
  //Transfer the inverted byte through the user-defined function.
  if (_sendFunction)
  {
    _sendFunction(0xFF ^ data);
  }
}

bool TLBLib::wait_ENA(bool state, unsigned long timeout)
{  
  //Start counting microseconds.
  unsigned long timeout_timer = micros();
  
  //Check the ENA line state while the timeout time hasn't passed.
  while (micros() - timeout_timer <= timeout) {
    //If the requested state is detected, return true.
    if (digitalRead(_ENA_pin) == state) {
      return true;
    }
  }
  
  //If the timeout time has passed, return false.
  return false;
}

void TLBLib::start_ENA()
{
  //Pull the ENA pin high.
  //Setting the pin HIGH before setting it as an OUTPUT prevents bouncing by
  //enabling the pull-up resistor first. 
  digitalWrite(_ENA_pin, HIGH);
  pinMode(_ENA_pin, OUTPUT);
}

void TLBLib::stop_ENA()
{
  //Pull the ENA pin low, then let go of it.
  digitalWrite(_ENA_pin, LOW);
  pinMode(_ENA_pin, INPUT);
}

TLBLib::sendError TLBLib::exit_send(sendError ret_value)
{
  //Ensure a distance of at least MIN_MESSAGE_RATE_ms between messages.
  
  //If send() finished successfully, check if more delay is necessary.
  if (ret_value == SUCCESS)
  {
    //If RPT_TIMEOUT_ms is greater than MIN_MESSAGE_RATE_ms, it's not necessary to wait anymore.
    if (MIN_MESSAGE_RATE_ms > RPT_TIMEOUT_ms)
    {
      delay(MIN_MESSAGE_RATE_ms - RPT_TIMEOUT_ms);
    }
  }
  //Otherwise, wait MIN_MESSAGE_RATE_ms.
  else
  {
    delay(MIN_MESSAGE_RATE_ms);
  }
  
  //Save the time of the last message sending attempt, so special messages also keep their distance.
  message_rate_timer = millis();
  
  //Return what was provided.
  return ret_value;
}
