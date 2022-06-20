#include <systemlog.h>
#include <Arduino.h>

Systemlog systemlog;

Systemlog::Systemlog() {
    memset(systemlog_buf, 0, 1024);
    cur_systemlog_idx = 0;
    num_logs = 0;
}

void Systemlog::log(char *m) {
    strncpy(systemlog_buf[cur_systemlog_idx++], m, SYSTEMLOG_ENTRY_SIZE); 
    if (cur_systemlog_idx >= SYSTEMLOG_MAX_ENTRIES) cur_systemlog_idx = 0;
    if (num_logs < SYSTEMLOG_MAX_ENTRIES) num_logs++;
}

bool Systemlog::get_log(unsigned char i, char *buf) {
    int pos = i;
    if (num_logs == SYSTEMLOG_MAX_ENTRIES) { // could have wrapped around after filling up ring buffer
        pos += cur_systemlog_idx; // move to position in ring buffer
        if (pos >= num_logs) pos -= num_logs;
    }

    if (i < SYSTEMLOG_MAX_ENTRIES && strlen(systemlog_buf[i]) > 0) {
        strncpy(buf, systemlog_buf[pos], SYSTEMLOG_ENTRY_SIZE);
        return true;
    }
    return false;
}

unsigned char Systemlog::get_num_logs() {
    return num_logs;
}
