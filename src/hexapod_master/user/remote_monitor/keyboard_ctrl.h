#ifndef KEYBOARDCONTR_H
#define KEYBOARDCONTR_H
#include<iostream>
#include<thread>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <linux/serial.h>
#include <termios.h>
#include <asm-generic/ioctl.h>
#include <sys/ioctl.h>
extern int KEYBOARD_CONTINUE_MODE;
class KeyBoardControl
{   
    private:

        struct termios _new_settings;
        struct termios _stored_settings;
        int _key = 0, _key_last = 0 , _key_output = 0;
    public:

        KeyBoardControl()
        {
            tcgetattr(0,&_stored_settings);
            _new_settings = _stored_settings;
            _new_settings.c_lflag &= (~ICANON);
            _new_settings.c_cc[VTIME] = 0;
            tcgetattr(0,&_stored_settings);
            _new_settings.c_cc[VMIN] = 1;//
            tcsetattr(0,TCSANOW,&_new_settings);
        }

        int scanKeyValue()
        {   
            _key = getchar();
            // _dotkey = getchar();

            printf("\nread a char : '%c'  %d   A ascii:%d \n", _key,_key,'a');
            return _key_output;
        }

        int retKeyValue_movemode()
        {                    
            return _key_output;
        }


        int retKeyValue()
        {                     //|| KEYBOARD_CONTINUE_MODE==1
            if(_key!=_key_last  )
            {
                _key_last=_key;
                _key_output=_key;
            }
            else if(KEYBOARD_CONTINUE_MODE==0 )
            {
                _key=0;
                _key_output=0;
            }

            if( KEYBOARD_CONTINUE_MODE==1)
            {
                _key_output=_key_last;
            }



            // if(_dotkey!=_dotkey_last )
            // {
            //     _dotkey_last=_dotkey;
            //     _dotkey_output=_dotkey;
            // }
            // else 
            // {
            //     _dotkey_output=0;
            // }

            // if( KEYBOARD_CONTINUE_MODE==0)
            // {
            //     _key_output=0;
            // }

            return _key_output;
        }

};


#endif
