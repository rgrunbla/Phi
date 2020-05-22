#ifndef CLOCK_H
#define CLOCK_H

class Clock {
    public:
        Clock();
        double get_time() const;
        void set_time(double time);
    
        double get_cursor() const;
        void set_cursor(double cursor);
    
    private:
        /* Time in the simulation */
        double time;

        /* Time we have to advance to */
        double cursor;
};

#endif // CLOCK_H