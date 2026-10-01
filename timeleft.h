#ifndef TIMELEFT_H
#define TIMELEFT_H

#include <algorithm>
#include <chrono>
#include <vector>

std::chrono::duration<double> time_left(
                                        std::chrono::steady_clock::time_point start ,
                                        std::chrono::steady_clock::time_point current ,
                                        const std::vector<int>& lines_per_thread ,
                                        const std::vector<int>& completed_per_thread){

    if (lines_per_thread.size() != completed_per_thread.size()) {
        return std::chrono::duration<double>::zero();
    }

    std::chrono::duration<double> elapsed = current - start;
    double estimated_remaining_seconds = 0.0;

    for (std::size_t i = 0; i < lines_per_thread.size(); ++i) {
        int total = lines_per_thread[i];
        int completed = completed_per_thread[i];
        if (total <= 0 || completed <= 0) {
            continue;
        }

        int lines_remaining = total - completed;
        double thread_estimate = elapsed.count() * lines_remaining / completed;
        estimated_remaining_seconds = std::max(estimated_remaining_seconds, thread_estimate);
    }

    return std::chrono::duration<double>(estimated_remaining_seconds);
    
}


#endif