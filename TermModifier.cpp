
#include "TermModifier.h"

TermModifier::TermModifier(unsigned int code) : codes({ code }) {}
TermModifier::TermModifier(std::vector<unsigned int>&& codes) : codes(std::move(codes)) {}

std::ostream& operator<<(std::ostream& out, const TermModifier& modifier) {
    out << "\033[" << modifier.codes[0];
    for (auto it = modifier.codes.cbegin() + 1; it < modifier.codes.cend(); it++) {
        out << ";" << *it;
    }
    return out << "m";
}

TermModifier TermModifier::operator|(const TermModifier& other) const {
    std::vector<unsigned int> newCodes;
    newCodes.reserve(codes.size() + other.codes.size());
    newCodes.insert(newCodes.end(), codes.begin(), codes.end());
    newCodes.insert(newCodes.end(), other.codes.begin(), other.codes.end());

    return TermModifier(std::move(newCodes));
}

TermModifier VT::Reset = TermModifier(0);
TermModifier VT::Bold = TermModifier(1);
TermModifier VT::Dim = TermModifier(2);
TermModifier VT::Underline = TermModifier(4);
TermModifier VT::Blink = TermModifier(5);
TermModifier VT::Invert = TermModifier(7);
TermModifier VT::Hide = TermModifier(8);

TermColor VT::Default = TermColor(9);
TermColor VT::Black = TermColor(0);
TermColor VT::Red = TermColor(1);
TermColor VT::Green = TermColor(2);
TermColor VT::Yellow = TermColor(3);
TermColor VT::Blue = TermColor(4);
TermColor VT::Magenta = TermColor(5);
TermColor VT::Cyan = TermColor(6);
TermColor VT::LightGray = TermColor(7);
TermColor VT::DarkGray = TermColor(60);
TermColor VT::LightRed = TermColor(61);
TermColor VT::LightGreen = TermColor(62);
TermColor VT::LightYellow = TermColor(63);
TermColor VT::LightBlue = TermColor(64);
TermColor VT::LightMagenta = TermColor(65);
TermColor VT::LightCyan = TermColor(66);
TermColor VT::White = TermColor(67);

VT::Foreground::Foreground(const TermColor& color) : TermModifier(color.code + 30) {}
VT::Background::Background(const TermColor& color) : TermModifier(color.code + 40) {}

TermColor::TermColor(unsigned int code) : code(code) {}
