//{ R, G, B }
#if defined(PINK)
#define RD 255
#define GR 0
#define BL 96

#elif defined(YELLOW)
#define RD 255
#define GR 176
#define BL 0

#elif defined(BLUE)
#define RD 125
#define GR 249
#define BL 255

#elif defined(CUSTOM_COLOUR)
#define RD 125
#define GR 128
#define BL 255

#else
#define RD -1
#define GR -1
#define BL -1
#endif
