#ifndef TIMELEFT_H
#define TIMELEFT_H

#include <chrono>

std::chrono::duration<double> time_left(
                                        std::chrono::steady_clock::time_point start ,
                                        std::chrono::steady_clock::time_point current , 
                                        int total_lines ,
                                        int lines_completed){

    if (lines_completed == 0 || lines_completed >= total_lines) {
        // Avoid division by zero or negative remaining lines
        return std::chrono::duration<double>::zero();
    }

    std::chrono::duration<double> elapsed = current - start;
    double average_time_per_line = elapsed.count() / lines_completed;
    int lines_remaining = total_lines - lines_completed;
    double estimated_remaining_seconds = average_time_per_line * lines_remaining;

    return std::chrono::duration<double>(estimated_remaining_seconds);
    
}


#endif