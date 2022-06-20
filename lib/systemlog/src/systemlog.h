#ifndef __SYSTEMLOG_H__
#define __SYSTEMLOG_H__

#define SYSTEMLOG_MAX_ENTRIES 20
#define SYSTEMLOG_ENTRY_SIZE 64

class Systemlog {
    public:
        Systemlog();
        void log(char *m);
        bool get_log(unsigned char i, char *buf);
        unsigned char get_num_logs();

    private:
        char systemlog_buf[SYSTEMLOG_MAX_ENTRIES][SYSTEMLOG_ENTRY_SIZE];
        unsigned char cur_systemlog_idx;
        unsigned char num_logs;
};

#endif
