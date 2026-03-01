// Choose your board.
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
#define EPD_PAINTER_PRESET_M5PAPER_S3


#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_presets.h"

EPD_Painter epd(EPD_PAINTER_PRESET);

// Terminal configuration
#define TERMINAL_FONT_SIZE  5
#define CHAR_WIDTH          6
#define CHAR_HEIGHT         20
#define LINE_SPACING        30
#define LINE_HEIGHT         (CHAR_HEIGHT + LINE_SPACING)
#define MARGIN_X            4
#define MARGIN_Y            4
#define MAX_ROWS            30

String termBuffer[MAX_ROWS];
int termRows = 0;

const char* sentences[] = {
  "Electronic paper displays reflect ambient light just like real paper.",
  "Unlike LCD screens, EPDs consume power only when the image changes.",
  "The bistable nature of e-ink means the image persists without any power.",
  "Microcapsules filled with black and white particles create the pixels.",
  "An electric field drives charged particles to the top or bottom of each capsule.",
  "EPDs are exceptionally readable in bright sunlight and outdoor conditions.",
  "Refresh rates are slow, typically between one and four seconds for a full update.",
  "Partial refresh modes can update regions of the screen much faster.",
  "Grey levels are achieved by precisely controlling particle positioning.",
  "Most consumer e-readers use a variant of active matrix e-ink technology.",
  "The low power draw makes EPDs ideal for battery powered embedded devices.",
  "E Ink Holdings is the dominant manufacturer of electrophoretic display film.",
  "Colour EPDs use a filter layer but sacrifice contrast and colour saturation.",
  "The viewing angle of an EPD is nearly identical in all directions.",
  "Ghost images can persist after refresh if the waveform is not tuned correctly.",
  "Temperature affects EPD refresh speed significantly in cold environments.",
  "Waveforms are lookup tables that control voltage sequences during a refresh.",
  "Driving an EPD incorrectly can permanently damage the display film.",
  "SPI is the most common interface used between a microcontroller and an EPD.",
  "Most EPD modules include a small RAM buffer to hold the current frame.",
  "A full refresh cycles the display through black and white to clear ghosting.",
  "The Adafruit GFX library provides a convenient abstraction for drawing on EPDs.",
  "Resolution on small EPDs typically ranges from 200 by 200 to 400 by 300 pixels.",
  "Flexible EPDs can be bent around curved surfaces for novel form factors.",
  "Electronic shelf labels in supermarkets are one of the largest EPD use cases.",
  "Smart badges and conference name tags increasingly use EPD panels.",
  "EPDs have an extremely long shelf life compared to LCD or OLED panels.",
  "The paper-like appearance reduces eye strain during extended reading sessions.",
  "Early e-ink prototypes were developed at the MIT Media Lab in the 1990s.",
  "Triton colour e-ink adds a colour filter array over a standard monochrome panel.",
  "Kaleido is a newer colour EPD technology with improved saturation.",
  "ACeP uses cyan magenta yellow and white particles for full colour reproduction.",
  "Spectra displays add a red or yellow pigment layer for three colour output.",
  "The ESP32 is a popular microcontroller choice for driving small EPD modules.",
  "Deep sleep between updates can reduce system power to just a few microamps.",
  "An EPD displaying a static image consumes exactly zero milliwatts.",
  "The contrast ratio of a good EPD rivals that of printed text on paper.",
  "Plastic Logic manufactures flexible EPD backplanes for rugged applications.",
  "EPD modules often include onboard voltage boosters for the high drive voltages.",
  "Typical EPD panel voltages range from fifteen to thirty volts during switching.",
  "The thin film transistor backplane controls each individual pixel electrode.",
  "Fast mode waveforms trade image quality for speed in dynamic applications.",
  "Some EPDs support sixteen or more grey levels for image rendering.",
  "Dithering is commonly used to simulate smooth gradients on EPD panels.",
  "The lack of backlight makes EPDs useless in complete darkness without a frontlight.",
  "Onyx Boox and Kindle are among the most recognised EPD based reading devices.",
  "Developers often wrap EPD drivers in the Adafruit GFX API for portability.",
  "This very screen you are reading is an electrophoretic display.",
  "Each refresh of this display is a small miracle of electrochemistry.",
  "And yet here we are, scrolling lorem ipsum replaced with EPD facts."
};
const int sentenceCount = sizeof(sentences) / sizeof(sentences[0]);
int sentenceIndex = 0;

String generateLine() {
  int cols = 35;
  String sentence = sentences[sentenceIndex % sentenceCount];
  sentenceIndex++;

  // Word-wrap: if the sentence fits, return it directly
  if ((int)sentence.length() <= cols) return sentence;

  // Otherwise truncate to the last whole word that fits
  String line = sentence.substring(0, cols);
  int lastSpace = line.lastIndexOf(' ');
  if (lastSpace > 0) line = line.substring(0, lastSpace);
  return line;
}

void renderTerminal() {
  epd.fillScreen(0);
  epd.setTextColor(3);
  epd.setTextSize(TERMINAL_FONT_SIZE);
  epd.setTextWrap(false);

  for (int i = 0; i < termRows-1; i++) {
    if (termBuffer[i].length() == 0) continue;
    epd.setCursor(MARGIN_X, MARGIN_Y + i * LINE_HEIGHT);
    epd.print(termBuffer[i]);
  }

  for (int i=0; i<35; i++){
    epd.fillRect(0, 450, 960, 90, 0);
    epd.setCursor(MARGIN_X, MARGIN_Y + (termRows-1) * LINE_HEIGHT);
    epd.print(termBuffer[(termRows-1)].substring(0,i));
    epd.print("_");
    epd.paint(1);
  }

}

void setup() {
  Serial.begin(115200);
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1);
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
  epd.clear();
  epd.clear();

  termRows = (epd.height() - MARGIN_Y * 2) / LINE_HEIGHT;
  if (termRows > MAX_ROWS) termRows = MAX_ROWS;

  Serial.print("Terminal size: ");
  Serial.print((epd.width() - MARGIN_X * 2) / CHAR_WIDTH);
  Serial.print(" cols x ");
  Serial.print(termRows);
  Serial.println(" rows");

  for (int i = 0; i < termRows; i++) termBuffer[i] = "";

  // Pre-fill so screen looks full from the start
  for (int i = 0; i < termRows; i++) {
    termBuffer[i] = generateLine();
  }

  renderTerminal();
}

void loop() {
  // Scroll all lines up by one
  for (int i = 0; i < termRows - 1; i++) {
    termBuffer[i] = termBuffer[i + 1];
  }
  // Add a new line at the bottom
  termBuffer[termRows - 1] = generateLine();

  long start = esp_timer_get_time();
  renderTerminal();
  //Serial.println(esp_timer_get_time() - start);
}