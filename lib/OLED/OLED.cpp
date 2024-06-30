// OLED.cpp
#include "OLED.h"

using namespace OLED;

Point::Point(int px, int py)
{
	x = px;
	y = py;
}

OLED::Point OLED::rotatePoint(const OLED::Point p, const int cx, const int cy , const float rads)
{
	return Point(	std::round(cos(rads) * (p.x - cx) - sin(rads) * (p.y - cy) + cx),
                  	std::round(sin(rads) * (p.x - cx) + cos(rads) * (p.y - cy) + cy));
}

DisplayHandler::DisplayHandler() {
}

void DisplayHandler::Initialise() {
	display.begin(0x3D, true); // Address 0x3D default
	display.setTextSize(2); // 2:1 pixel scale
	display.setTextColor(SH110X_WHITE); // Tells display to draw white text
	display.setTextColor(SH110X_WHITE, SH110X_BLACK); //sets the text background colour to black to overwrite existing numbers, prevents flicker. 
	//display.setContrast (0); // dim display (optional)
	display.clearDisplay();
	display.display();
}

void DisplayHandler::drawCompass() {
	display.fillRect(0, 0, SCREEN_WIDTH, 28, SH110X_WHITE);
	display.drawLine(cx-line_length,cy, cx+line_length, cy,SH110X_WHITE);
	display.drawLine(cx,cy-line_length, cx, cy+line_length,SH110X_WHITE);

	// North Triangle
	display.fillTriangle( 	cx,cy+line_length,
							cx-5,cy+line_length-5,
							cx+5,cy+line_length-5,
							SH110X_WHITE);

	// South Triangle
	display.fillTriangle( 	cx,cy-line_length,
							cx-5,cy-line_length+5,
							cx+5,cy-line_length+5,
							SH110X_WHITE);

	// East Triangle
	display.fillTriangle( 	cx+line_length,cy,
							cx+line_length-5,cy-5,
							cx+line_length-5,cy+5,
							SH110X_WHITE);

  // West Triangle
	display.fillTriangle( 	cx-line_length,cy,
							cx-line_length+5,cy-5,
							cx-line_length+5,cy+5,
							SH110X_WHITE);
}

void DisplayHandler::drawCompassDirection(CompassDirection direction)
{
	int end_x,end_y;
	float angle;
	end_x = cx;
	end_y = line_length;
	switch(direction)
	{
		case NORTH:
		angle = 0;
		// end_x = cx;
		// end_y = cy+line_length;
		break;

		case NORTH_EAST:
		angle = M_PI_4;
		// end_x = cx+line_length/M_SQRT2;
		// end_y = cy+line_length/M_SQRT2;
		break;

		case EAST:
		angle = 2*M_PI_4;
		// end_x = cx;
		// end_y = cy+line_length;
		break;

		case SOUTH_EAST:
		angle = 3*M_PI_4;
		// end_x = cx+line_length/M_SQRT2;
		// end_y = cy-line_length/M_SQRT2;
		break;

		case SOUTH:
		angle = 4*M_PI_4;
		// end_x = cx;
		// end_y = cy-line_length;
		break;

		case SOUTH_WEST:
		angle = 5*M_PI_4;
		// end_x = cx-line_length/M_SQRT2;
		// end_y = cy-line_length/M_SQRT2;
		break;

		case WEST:
		angle = 6*M_PI_4;
		// end_x = cx;
		// end_y = cy-line_length;
		break;

		case NORTH_WEST:
		angle = 7*M_PI_4;
		// end_x = cx-line_length/M_SQRT2;
		// end_y = cy+line_length/M_SQRT2;
		break;
	}

	Point p_end(end_x,end_y);
	Point p_rotated = rotatePoint(p_end,cx,cy,angle);

	display.drawLine(cx, cy, p_rotated.x, p_rotated.y,SH110X_WHITE);
}

void DisplayHandler::Distance(double distance) {
  display.fillRect(70, 30, 58, 98, SH110X_BLACK); //clears the old distance value
  display.setTextSize(3);
  display.setCursor(0, 34);
  display.print(distance);
  display.println("m");
  // display.display();
}

void DisplayHandler::Compass(double compass) {
  display.fillRect(70, 70, 58, 58, SH110X_BLACK); //clears the old compass value
  display.setTextSize(3);
  display.setCursor(0, 70);
  display.print(compass);
  display.print((char)247); // degree symbol 
  // display.display();
}

void DisplayHandler::Clino(double clino) {
  display.fillRect(70, 106, 58, 22, SH110X_BLACK); //clears the old clino value
  display.setTextSize(3);
  display.setCursor(0, 106);
  display.print(clino);
  display.print((char)247); // degree symbol 
  // display.display();
}

void DisplayHandler::Sensor_cal_status(int sensor_status) {
  display.setTextSize(2);
  display.setCursor(23, 4);
  display.print(sensor_status);
  // display.display();
}

void DisplayHandler::Blutooth(bool ble_status)
{
  // insert switch case to update
  const unsigned char PROGMEM Bluetooth_icon[] = {
      0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x01, 0xfe, 0x00, 0x07, 0xff, 0x00, 0x07, 0xcf, 0x80, 0x0f,
      0xc7, 0xc0, 0x0f, 0xc3, 0xc0, 0x0e, 0xd9, 0xc0, 0x0f, 0x03, 0xc0, 0x0f, 0x87, 0xc0, 0x0f, 0x87,
      0xc0, 0x0f, 0xcf, 0xc0, 0x0f, 0x87, 0xc0, 0x0f, 0x03, 0xc0, 0x0e, 0x59, 0xc0, 0x0f, 0xc3, 0xc0,
      0x0f, 0xc7, 0xc0, 0x07, 0xcf, 0x80, 0x07, 0xff, 0x80, 0x03, 0xff, 0x00, 0x00, 0xfe, 0x00, 0x00,
      0x00, 0x00};

  if (ble_status == false)
  {
    display.fillRect(0, 0, 22, 22, SH110X_BLACK); // clears the blutooth symbol
    display.display();
  }
  else
  {
    display.drawBitmap(0, 0, Bluetooth_icon, 22, 22, 10, SH110X_WHITE);
    display.display();
  }
}

void DisplayHandler::Battery(int batt_percentage) {
  display.drawRect(90, 0, 32, 15, SH110X_WHITE);
  display.drawRect(122, 4, 3, 6, SH110X_WHITE);
  batt_level = (batt_percentage/100.00)*32;
  display.fillRect(90, 0, batt_level, 15, SH110X_WHITE);
  display.fillRect(91+batt_level, 1, 30-batt_level, 13, SH110X_BLACK);
  display.setTextSize(2);
  display.setCursor(50, 0);
  display.print(batt_percentage);
  display.print("%");
  display.display();
}

void DisplayHandler::clearDisplay() {
   display.clearDisplay();
}

void DisplayHandler::update() {
   display.display();
}