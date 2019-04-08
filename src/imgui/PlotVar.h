//
// Created by Abiel on 1/10/19.
//

#ifndef ECTOJERKTESTS_PLOTVAR_H
#define ECTOJERKTESTS_PLOTVAR_H


namespace ImGui {
    // Plot value over time
// Pass FLT_MAX value to draw without adding a new value
    void	PlotVar(const char* label, float value, float scale_min = FLT_MAX, float scale_max = FLT_MAX, size_t buffer_size = 120);

// Call this periodically to discard old/unused data
    void	PlotVarFlushOldEntries();
};


#endif //ECTOJERKTESTS_PLOTVAR_H
