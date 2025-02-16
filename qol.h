#ifndef QOL_H
#define QOL_H

#ifdef QOL
#define printcmd(msg, array, len) do { \
    fprintf(stderr, msg); \
    for (int i = 0; i < len; ++i) { \
        fprintf(stderr, " %02X", array[i]); \
    } \
    fprintf(stderr, "\n"); \
} while (0);
#else
#define printcmd(msg, array, len) do {} while (0);
#endif

#endif // QOL_H
