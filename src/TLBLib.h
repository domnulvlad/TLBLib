#ifndef TLBLib_H
#define TLBLib_H

#include <Arduino.h>

class TLBLib
{
  public:
    //Function pointer types for callbacks
    using beginFunction_type = void (*)(void);          //beginFunction
    using endFunction_type   = void (*)(void);          //endFunction
    using sendFunction_type  = void (*)(uint8_t data);  //sendFunction
    using errorFunction_type = void (*)(unsigned long); //errorFunction
    
    //Return values for the send() function
    enum sendError
    {
      FAIL,    //the message failed to send
      SUCCESS, //the message was sent successfully
      REPEAT   //the message was sent, but it must be repeated
    };
    
    //Constructor
    TLBLib(uint8_t ENA_pin, sendFunction_type sendFunction, beginFunction_type beginFunction = nullptr, endFunction_type endFunction = nullptr);
    
    //Rates
    unsigned long KEEP_ALIVE_RATE_ms  = 1000; //millisecond interval for sending "keep alive" messages
    unsigned long MIN_MESSAGE_RATE_ms = 5;    //minimum millisecond time to wait after sending a message
    
    //Timeouts
    unsigned long RPT_TIMEOUT_ms = 6;   //milliseconds to wait for a "repeat" pulse after sending a message
    unsigned long REQ_TIMEOUT_us = 200; //microseconds to wait for ENA to go HIGH
    unsigned long ACK_TIMEOUT_us = 200; //microseconds to wait for ENA to go LOW
    
    //Set a function ("void errorFunction(unsigned long duration)") to be executed when an error is detected
    void errorFunction(errorFunction_type function);
    
    //Initialize the bus
    void begin();
    //Deinitialize the bus
    void end();
    
    //Maintain and monitor the connection
    void update();
    
    void turnOff();
    
    //Attempt to send a message
    sendError send(uint8_t* data);
  
  private:
    //Enable line
    uint8_t _ENA_pin;
    
    //Callback functions
    sendFunction_type  _sendFunction;
    beginFunction_type _beginFunction;
    endFunction_type   _endFunction;
    errorFunction_type _errorFunction;
    
    //Timers
    unsigned long keep_alive_timer;
    unsigned long message_rate_timer;
    
    //Opcodes
    static const uint8_t TURN_OFF_OPCODE   = 0xC0;
    static const uint8_t KEEP_ALIVE_OPCODE = 0xC3;
    
    //Maintain the connection
    void keep_alive();
    
    //Check for errors
    void check_error_pulse();
    
    //Start a packet or send a single-byte message
    bool send_opcode(uint8_t opcode);
    
    //Transfer a byte on the bus
    void send_byte(uint8_t data, uint8_t* crc = nullptr);
    
    //Wait for a specific state on ENA
    bool wait_ENA(bool state, unsigned long timeout);
    
    //Pull the ENA line
    void start_ENA();
    //Let go of the ENA line
    void stop_ENA();
    
    //Exit the send() function
    sendError exit_send(sendError ret_value);
};

#endif
