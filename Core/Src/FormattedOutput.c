#include "FormattedOutput.h"

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

void NumToStr(uint32_t numIn, uint8_t* stringOut, uint8_t sizeIn)
{
    uint32_t digits = digitCount_uint32_t(numIn);
    if(digits<sizeIn)
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
