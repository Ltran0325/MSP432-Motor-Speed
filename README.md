# MSP432-Motor-Speed

## Description:

Control motor speed using a potentiometer. Potentiometer voltage readings are passed through a digital low pass filter to smoothen speed transitions. Speed in RPM is displayed on a seven-segment display.

## Demo:

https://youtu.be/RyNKAnJRMWo

## Calculations:

### Low-Pass Filter Transfer Function:

<a href="https://www.codecogs.com/eqnedit.php?latex=H(s)&space;=&space;\frac{1}{1&plus;\frac{s}{w}}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?H(s)&space;=&space;\frac{1}{1&plus;\frac{s}{w}}" title="H(s) = \frac{1}{1+\frac{s}{w}}" /></a>

### MATLAB Program:

```MATLAB

Hc = tf([1], [1/(2*pi*1), 1]);

bode(Hc);

Hd = c2d(Hc, 1/200);

```

### MATLAB Result:

```MATLAB

   0.03093
  ----------
  z - 0.9691
  
```
  
### Find Output 'y':
  
<a href="https://www.codecogs.com/eqnedit.php?latex=H(z)&space;=&space;\frac{Y(z)}{X(z)}&space;=&space;\frac{b_0}{z&plus;a_0}&space;=&space;\frac{0.03903}{z-0.9691}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?H(z)&space;=&space;\frac{Y(z)}{X(z)}&space;=&space;\frac{b_0}{z&plus;a_0}&space;=&space;\frac{0.03903}{z-0.9691}" title="H(z) = \frac{Y(z)}{X(z)} = \frac{b_0}{z+a_0} = \frac{0.03903}{z-0.9691}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=Y(z)(z&plus;a_0)&space;=&space;X(z)(b_0)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Y(z)(z&plus;a_0)&space;=&space;X(z)(b_0)" title="Y(z)(z+a_0) = X(z)(b_0)" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=y(k)&space;&plus;&space;a_0y(k-1)&space;=&space;b_0x(k-1)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y(k)&space;&plus;&space;a_0y(k-1)&space;=&space;b_0x(k-1)" title="y(k) + a_0y(k-1) = b_0x(k-1)" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=y(k)&space;=&space;-a_0y(k-1)&space;&plus;&space;b_0x(k-1)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y(k)&space;=&space;-a_0y(k-1)&space;&plus;&space;b_0x(k-1)" title="y(k) = -a_0y(k-1) + b_0x(k-1)" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=y(k)&space;=&space;0.9691y(k-1)&space;&plus;&space;0.03903x(k-1)" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y(k)&space;=&space;0.9691y(k-1)&space;&plus;&space;0.03903x(k-1)" title="y(k) = 0.9691y(k-1) + 0.03903x(k-1)" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=y&space;=&space;0.9691y_1&space;&plus;&space;0.03903x_1" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y&space;=&space;0.9691y_1&space;&plus;&space;0.03903x_1" title="y = 0.9691y_1 + 0.03903x_1" /></a>
