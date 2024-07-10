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

OLED::DisplayHandler::DisplayHandler() {
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

void DisplayHandler::drawCentered(String str, int cx, int cy, int size)
{
	display.setTextSize(size);
	display.setCursor(cx - (int)((str.length()/2.0)*size*6), cy-size*6/2);
	display.print(str);
}

void DisplayHandler::drawCalib(CompassDirection pointing, CompassDirection facing)

{
	String str;
	display.fillRect(0, 0, SCREEN_WIDTH, 28, SH110X_BLACK);
	drawCentered("CALIB",SCREEN_WIDTH/2,7,2);

	drawCentered("Pointing",SCREEN_WIDTH/4,25,1);
	drawCentered(directionsArr[(int)pointing],SCREEN_WIDTH/4,40,1);

	drawCompassDirection(SCREEN_WIDTH/4,90,28,3,pointing);

	drawCentered("Facing",3*SCREEN_WIDTH/4,25,1);
	drawCentered(directionsArr[(int)facing],3*SCREEN_WIDTH/4,40,1);

	drawCompassDirection(3*SCREEN_WIDTH/4,90,28,3,facing);
}

void DisplayHandler::drawCompass(int cx, int cy, int line_length, int arrow_length) {
	// display.fillRect(0, 0, SCREEN_WIDTH, 28, SH110X_WHITE);
	display.drawLine(cx-line_length,cy, cx+line_length, cy,SH110X_WHITE);
	display.drawLine(cx,cy-line_length, cx, cy+line_length,SH110X_WHITE);

	// North Triangle
	display.fillTriangle( 	cx,cy+line_length,
							cx-arrow_length,cy+line_length-arrow_length,
							cx+arrow_length,cy+line_length-arrow_length,
							SH110X_WHITE);

	// South Triangle
	display.fillTriangle( 	cx,cy-line_length,
							cx-arrow_length,cy-line_length+arrow_length,
							cx+arrow_length,cy-line_length+arrow_length,
							SH110X_WHITE);

	// East Triangle
	display.fillTriangle( 	cx+line_length,cy,
							cx+line_length-arrow_length,cy-arrow_length,
							cx+line_length-arrow_length,cy+arrow_length,
							SH110X_WHITE);

  // West Triangle
	display.fillTriangle( 	cx-line_length,cy,
							cx-line_length+arrow_length,cy-arrow_length,
							cx-line_length+arrow_length,cy+arrow_length,
							SH110X_WHITE);
}

void DisplayHandler::drawCompassDirection(int cx, int cy, int line_length, int arrow_length, CompassDirection direction)
{
	float angle;
	float N_arrow = 1;
	switch(direction)
	{
		case UP:
		angle = 0;
		N_arrow = 1;
		break;

		case DOWN:
		angle = 180;
		N_arrow = 1;
		break;

		case NORTH:
		angle = 0;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case NORTH_EAST:
		angle = M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case EAST:
		angle = 2*M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case SOUTH_EAST:
		angle = 3*M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case SOUTH:
		angle = 4*M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case SOUTH_WEST:
		angle = 5*M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case WEST:
		angle = 6*M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;

		case NORTH_WEST:
		angle = 7*M_PI_4;
		N_arrow = 3;
		drawCompass(cx,cy,line_length,arrow_length);
		break;
		
	}


	Point p_default(cx,cy-line_length);
	Point p_start(cx,cy);
	Point p_end(cx,cy-line_length);
	p_end = rotatePoint(p_end,cx,cy,angle);
	display.drawLine(p_start.x, p_start.y, p_end.x, p_end.y,SH110X_WHITE);


	for (int i=0; i<N_arrow; i++)
	{
		p_start.x = p_default.x;
		p_start.y = p_default.y + i*arrow_length;
		p_end.x = p_start.x - arrow_length;
		p_end.y = p_start.y + arrow_length;
		p_end = rotatePoint(p_end,cx,cy,angle);
		p_start = rotatePoint(p_start,cx,cy,angle);
		display.drawLine(p_start.x, p_start.y, p_end.x, p_end.y,SH110X_WHITE);

		p_start.x = p_default.x;
		p_start.y = p_default.y + i*arrow_length;
		p_end.x = p_start.x + arrow_length;
		p_end.y = p_start.y + arrow_length;
		p_end = rotatePoint(p_end,cx,cy,angle);
		p_start = rotatePoint(p_start,cx,cy,angle);
		display.drawLine(p_start.x, p_start.y, p_end.x, p_end.y,SH110X_WHITE);
	}
}

void DisplayHandler::displayYN(const char prompt[11], bool YN)
{

	const int prompt_height = canvas_center_y - 40;
	const int selector_height = canvas_center_y - 10;

	// String str(prompt);
	drawCentered(prompt,canvas_center_x,prompt_height,2);

	if (YN)
	{
		display.fillRect(	canvas_center_x - 60, selector_height - 15,
							50, 30,
							SH110X_WHITE);

		display.fillRect(	canvas_center_x - 58,selector_height - 13,
							46, 26,
							SH110X_BLACK);

		drawCentered("YES",canvas_center_x - 34, selector_height-1, 2);
		drawCentered("NO",canvas_center_x + 34, selector_height-1, 2);

	} else {

		display.fillRect(	canvas_center_x + 8, selector_height - 15,
							50, 30,
							SH110X_WHITE);

		display.fillRect(	canvas_center_x + 10, selector_height - 13,
							46, 26,
							SH110X_BLACK);

		drawCentered("YES",canvas_center_x - 34, selector_height-1, 2);
		drawCentered("NO",canvas_center_x + 34, selector_height-1, 2);
	}
}

void DisplayHandler::displayYN(const char prompt_top[11], const char prompt_btm[11], bool YN)
{

	const int prompt_height = canvas_center_y - 40;
	const int selector_height = canvas_center_y + 7;

	// String str(prompt);
	drawCentered(prompt_top,canvas_center_x,prompt_height,2);
	drawCentered(prompt_btm,canvas_center_x,prompt_height+18,2);

	if (YN)
	{
		display.fillRect(	canvas_center_x - 60, selector_height - 15,
							50, 30,
							SH110X_WHITE);

		display.fillRect(	canvas_center_x - 58,selector_height - 13,
							46, 26,
							SH110X_BLACK);

		drawCentered("YES",canvas_center_x - 34, selector_height-1, 2);
		drawCentered("NO",canvas_center_x + 34, selector_height-1, 2);

	} else {

		display.fillRect(	canvas_center_x + 8, selector_height - 15,
							50, 30,
							SH110X_WHITE);

		display.fillRect(	canvas_center_x + 10, selector_height - 13,
							46, 26,
							SH110X_BLACK);

		drawCentered("YES",canvas_center_x - 34, selector_height-1, 2);
		drawCentered("NO",canvas_center_x + 34, selector_height-1, 2);
	}
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