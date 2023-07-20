
#ifndef STOCHASTICGAMESCPP_TERMMODIFIER_H
#define STOCHASTICGAMESCPP_TERMMODIFIER_H


#include <ostream>
#include <vector>

class TermModifier {
    std::vector<unsigned int> codes;
public:
    TermModifier(unsigned int code);
    TermModifier(std::vector<unsigned int>&& codes);
    friend std::ostream& operator<<(std::ostream& out, const TermModifier& modifier);
    TermModifier operator|(const TermModifier& other) const;
};

class TermColor {
public:
    unsigned int code;

    TermColor(unsigned int code);
};

class VT {
public:
    static TermModifier Reset;
    static TermModifier Bold;
    static TermModifier Dim;
    static TermModifier Underline;
    static TermModifier Blink;
    static TermModifier Invert;
    static TermModifier Hide;

    static TermColor Default;
    static TermColor Black;
    static TermColor Red;
    static TermColor Green;
    static TermColor Yellow;
    static TermColor Blue;
    static TermColor Magenta;
    static TermColor Cyan;
    static TermColor LightGray;
    static TermColor DarkGray;
    static TermColor LightRed;
    static TermColor LightGreen;
    static TermColor LightYellow;
    static TermColor LightBlue;
    static TermColor LightMagenta;
    static TermColor LightCyan;
    static TermColor White;

    class Foreground : public TermModifier {
    public:
        Foreground(const TermColor& color);
    };

    class Background : public TermModifier {
    public:
        Background(const TermColor& color);
    };
};


#endif //STOCHASTICGAMESCPP_TERMMODIFIER_H
#pragma once
