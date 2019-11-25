
/** @file STMGood.h
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief data analysis and send message to the motor, used in pid debug mainly
 *
 */
#ifndef  _STMGOOD_H_
#define _STMGOOD_H_

#include "stm32f4xx.h"

int cmd(char *Cmd,int n);
void multi1(int n);
void multi2(int n);
void multi3(int n);
void multi4(int n);
void multi5(int n);
void multi6(int n);
void multi7(int n);
void multi8(int n);
void multi9(int n);

void part1(int xx);
void part2(int xx);
void part3(int xx);
void part4(int xx);
void part5(int xx);
void part6(int xx);

void go(void);
void back(void);
void left(void);
void right(void);
void goleft(void);
void goright(void);
void backleft(void);
void backright(void);
void stopcar(void);

void go1(void);
void back1(void);
void left1(void);
void right1(void);
void goleft1(void);
void goright1(void);
void backleft1(void);
void backright1(void);

void Dealdata(int Rx);
void sendint1(int x);
void sendint2(int x);
void sendint3(int x);
void sendint4(int x);
void sendint5(int x);
void sendint6(int x);
void sendint7(int x);
void sendint8(int x);
void sendint9(int x);
void sendint10(int x);
void senddouble1(double x);
void senddouble2(double x);
void senddouble3(double x);
void senddouble4(double x);
void senddouble5(double x);
void senddouble6(double x);
void senddouble7(double x);
void senddouble8(double x);
void senddouble9(double x);
void senddouble10(double x);
int Command(char *Cmd,int n);
void SplitStr(char *str,char *str1,char *str2);
int Strlen(char *str);
int CompStr(char *str1,char *str2);
int CopyStr(char *str1,char *str2);
int FirstSpace(char *str);
int DealStr(char *str);
int StrToInt(char *Str);
float StrToFloat(char *str);
char *SubStr(char *str,int start,int final);
extern float _kkp,_kki,_kkd;
extern float _kp,_ki,_kd;
extern float target,maxout1,maxout2;
#endif
