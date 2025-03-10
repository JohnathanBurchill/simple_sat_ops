#include "satellite_status.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int parse_satellite_status_file(char *filename, satellite_status_t **status_list, int *n_entries)
{
    if (status_list == NULL) {
        fprintf(stderr, "Expected non-null status_list pointer\n");
        return -1;
    }
    
    FILE *f = fopen(filename, "r"); 
    if (f == NULL) {
        fprintf(stderr, "Error opening satellite status file %s\n", filename);
        return -2;
    }

    // count number of lines not starting with a comment character '#'
    
    char line_buffer[256];
    int entries = 0;
    while (fgets(line_buffer, 256, f)) {
        if (line_buffer[0] == '#') {
            continue;
        }
        entries++;
    }; 
    
    if (n_entries != NULL) {
        *n_entries = entries;
    }
    *status_list = malloc(sizeof(satellite_status_t) * entries);
    if (*status_list == NULL) {
        fprintf(stderr, "Unable to allocate memory for satellite info list\n");
        fclose(f);
        return -3;
    }
    entries = 0;
    satellite_status_t *s = NULL;
    fseek(f, 0L, SEEK_SET);

    char *ap = NULL;
    size_t offset = 0;
    while (fgets(line_buffer, 256, f)) {
        if (line_buffer[0] == '#') {
            continue;
        }
        s = *status_list + entries;
        ap = line_buffer;
        for (int i = 0; i < 8; ++i) {
            while (*ap != ';' && *ap != '\n' && *ap != '\0') ++ap;
            *ap = '\0';
            ++ap;
        }
        offset = 0;
        snprintf(s->name, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->id, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->f_uplink_mhz, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->f_downlink_mhz, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->f_beacon_mhz, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->mode, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->callsign, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        snprintf(s->status, 64, "%s", line_buffer + offset);
        offset += strlen(line_buffer + offset) + 1;
        entries++;
    }; 

    fclose(f);

    return 0;
}
