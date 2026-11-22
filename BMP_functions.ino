// Bodmer's BMP image rendering function
const char *digit0File = "/0-neon.bmp";
const char *digit1File = "/1-neon.bmp";
const char *digit2File = "/2-neon.bmp";
const char *digit3File = "/3-neon.bmp";
const char *digit4File = "/4-neon.bmp";
const char *digit5File = "/5-neon.bmp";
const char *digit6File = "/6-neon.bmp";
const char *digit7File = "/7-neon.bmp";
const char *digit8File = "/8-neon.bmp";
const char *digit9File = "/9-neon.bmp";

uint16_t drawBmp(const char *filename, int16_t x, int16_t y) {

  if ((x >= tft.width()) || (y >= tft.height())) return 0;

  fs::File bmpFS;

  // Open requested file on SD card
  bmpFS = LittleFS.open(filename, "r");

  if (!bmpFS)
  {
    Serial.print("File not found");
    return 0;
  }
  if (bmpFS.size() <= 0)
  {
    Serial.print("*** FILE ERROR *** Empty File! *** FILE ERROR ***");
  }
 
  uint32_t seekOffset;
  uint16_t w, h, row, col;
  uint8_t  r, g, b;

  uint32_t startTime = millis();

  if (read16(bmpFS) == 0x4D42)
  {
    read32(bmpFS);
    read32(bmpFS);
    seekOffset = read32(bmpFS);
    read32(bmpFS);
    w = read32(bmpFS);
    h = read32(bmpFS);

    if ((read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0))
    {
      y += h - 1;

      bool oldSwapBytes = tft.getSwapBytes();
      tft.setSwapBytes(true);
      bmpFS.seek(seekOffset);

      uint16_t padding = (4 - ((w * 3) & 3)) & 3;
      uint8_t lineBuffer[w * 3 + padding];

      for (row = 0; row < h; row++) {
        
        bmpFS.read(lineBuffer, sizeof(lineBuffer));
        uint8_t*  bptr = lineBuffer;
        uint16_t* tptr = (uint16_t*)lineBuffer;
        // Convert 24 to 16 bit colours
        for (uint16_t col = 0; col < w; col++)
        {
          b = *bptr++;
          g = *bptr++;
          r = *bptr++;
          *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }

        // Push the pixel row to screen, pushImage will crop the line if needed
        // y is decremented as the BMP image is drawn bottom up
        #ifdef TFT_TRANSPARENT_COLOR
          tft.pushImage(x, y--, w, 1, (uint16_t*)lineBuffer,TFT_TRANSPARENT_COLOR);
        #else
          tft.pushImage(x, y--, w, 1, (uint16_t*)lineBuffer);
        #endif
      }
      tft.setSwapBytes(oldSwapBytes);
      Serial.print("Loaded in "); Serial.print(millis() - startTime);
      Serial.println(" ms");
    }
    else Serial.println("BMP format not recognized.");
  }
  bmpFS.close();
  return w;
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

uint16_t drawBMPDigit(uint8_t digit, int16_t x, int16_t y)
{
  switch(digit)
  {
    case 0:
      return drawBmp(digit0File, x, y);
    break;
    case 1:
      return drawBmp(digit1File, x, y);
    break;
    case 2:
      return drawBmp(digit2File, x, y);
    break;
    case 3:
      return drawBmp(digit3File, x, y);
    break;
    case 4:
      return drawBmp(digit4File, x, y);
    break;
    case 5:
      return drawBmp(digit5File, x, y);
    break;
    case 6:
      return drawBmp(digit6File, x, y);
    break;
    case 7:
      return drawBmp(digit7File, x, y);
    break;
    case 8:
      return drawBmp(digit8File, x, y);
    break;
    case 9:
      return drawBmp(digit9File, x, y);
    break;
  }
}
