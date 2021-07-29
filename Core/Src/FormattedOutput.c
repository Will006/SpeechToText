#include "FormattedOutput.h"

int16_t digitCount_int16_t(int16_t valIn)
{
    int16_t returnVal=0;
    do
    {
        valIn/=10;
        returnVal++;
    }while(valIn!=0);
    return returnVal;
}

uint32_t digitCount_uint32_t(uint32_t valIn)
{
    uint32_t returnVal=0;
    do
    {
        valIn/=10;
        returnVal++;
    }while(valIn!=0);
    return returnVal;
}

void NumToStr_uint32(uint32_t numIn, uint8_t* stringOut, uint8_t sizeIn)
{
    uint32_t digits = digitCount_uint32_t(numIn);
    if(digits<=sizeIn)
    {
        stringOut[digits]='\0';
        digits--;
        for(int i=digits; i>=0; i--)
        {

            stringOut[i]='0'+numIn%10;
            numIn/=10;
        }
    }
}

void NumToStr_int16(int16_t numIn, uint8_t* stringOut, uint8_t sizeIn)
{
    int16_t digits = digitCount_int16_t(numIn);
    if(digits<=sizeIn)
    {
        stringOut[digits]='\0';
        digits--;
        for(int i=digits; i>=0; i--)
        {

            stringOut[i]='0'+numIn%10;
            numIn/=10;
        }
    }
}

 