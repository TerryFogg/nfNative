//
// Copyright (c) 2017 The nanoFramework project contributors
// See LICENSE file in the project root for full license information.
//

//////////////////////////////////////////////////////////////////////////////
// This file was automatically generated by a tool.                         //
// Any changes you make here will be overwritten when it's generated again. //
//////////////////////////////////////////////////////////////////////////////

#ifndef SWO_H_
#define SWO_H_


#define SWO_OUTPUT FALSE


#if (SWO_OUTPUT == TRUE)

#ifdef __cplusplus
extern "C" {
#endif

    void SwoInit();
    void SwoPrintChar(char c);
    void SwoPrintString(const char *s);

#ifdef __cplusplus
}
#endif

#endif //(SWO_OUTPUT == TRUE)


#endif  // SWO_H_