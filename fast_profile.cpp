
#include "fast_profile.h"

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <map>
#include <chrono>
#include <iomanip>

class profile_node;
class call;

void profiling_start(std::string key);
void profiling_stop();
void profiling_tree(std::ostream& out, profile_node& node, int depth);

class call {
public:
    std::chrono::high_resolution_clock::time_point started;
    long elapsed;
    long self;

    call(const std::chrono::high_resolution_clock::time_point& started, long elapsed, long self) : started(started), elapsed(elapsed),
        self(self) {}
};

class profile_node {
public:
    int parent;
    std::string name;
    std::map<std::string, int> callee;

    std::list<call> calls;
    std::chrono::high_resolution_clock::time_point started;
    long nonself;

    profile_node(int parent, std::string name) : parent(parent), name(name) {
        start();
    }

    void start() {
        started = std::chrono::high_resolution_clock::now();
        nonself = 0;
    }

    long stop() {
        std::chrono::high_resolution_clock::time_point stopped = std::chrono::high_resolution_clock::now();
        long nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(stopped - started).count();

        calls.emplace_back(started, nanos, nanos - nonself);

        return nanos;
    }
};

thread_local static struct {
    int current = -1;
    int depth = -1;

    int counter = 0;
    std::vector<profile_node> calltree;
} __profiling;

void profiling_start(std::string key) {
    if (__profiling.current == -1) {
        __profiling.calltree.push_back(profile_node(-1, key));
        __profiling.current = __profiling.counter++;
    }
    else {
        auto& currentNode = __profiling.calltree[__profiling.current];
        auto& children = currentNode.callee;

        auto result = children.insert(std::map<std::string, int>::value_type(key, __profiling.counter));
        if (result.second) {
            __profiling.calltree.push_back(profile_node(__profiling.current, key));
            __profiling.current = __profiling.counter++;
        }
        else {
            __profiling.current = result.first->second;
            __profiling.calltree[__profiling.current].start();
        }
    }

    __profiling.depth++;
}

void profiling_stop() {
    auto& node = __profiling.calltree[__profiling.current];
    long elapsed = node.stop();

    __profiling.current = node.parent;
    if (__profiling.current != -1) {
        __profiling.calltree[__profiling.current].nonself += elapsed;
    }
    __profiling.depth--;
}

    const std::string css = "body {\n"
                            "  font: 9pt monospace;\n"
                            "}\n"
                            "details {\n"
                            "  margin-left: 8px;\n"
                            "  border-left: 1px solid black;\n"
                            "}\n"
                            "details summary::-webkit-details-marker {\n"
                            "  display:none;\n"
                            "}\n"
                            "details[open] > details:last-child {\n"
                            "  margin-bottom: 5px;\n"
                            "  border-bottom: 1px solid black;\n"
                            "}\n"
                            "details[open] > summary {\n"
                            "  background-color: green;\n"
                            "  color: white;\n"
                            "  margin-left: 8px;\n"
                            "  border-left: 1px solid black;\n"
                            "}\n"
                            "summary {\n"
                            "  margin-left: 9px;\n"
                            "  padding: 3px;\n"
                            "}\n"
                            "summary .count {\n"
                            "  display: inline-block;\n"
                            "  text-align: right;\n"
                            "  width: 50px;\n"
                            "  margin-right: 10px;\n"
                            "}\n"
                            "summary .name {\n"
                            "  font-weight: bold;\n"
                            "}\n"
                            "summary .time {\n"
                            "  float: right;\n"
                            "}\n"
                            "summary .time .self {\n"
                            "  display: inline-block;\n"
                            "  width: 100px;\n"
                            "  text-align: right;\n"
                            "}\n"
                            "summary .time .total {\n"
                            "  display: inline-block;\n"
                            "  width: 100px;\n"
                            "  text-align: right;\n"
                            "}\n"
                            "";

void profiling_info(std::ostream& out) {
    out << std::fixed << std::setprecision(2);
    out << "<html><head><style>" << css << "</style><body>";

    for (auto&& node : __profiling.calltree) {
        if (node.parent == -1) {
            profiling_tree(out, node, 0);
        }
    }

    out << "</body></html>";
}

profiling_handle::profiling_handle(std::string key) {
    profiling_start(key);
}

profiling_handle::~profiling_handle() {
    profiling_stop();
}

void profiling_tree(std::ostream& out, profile_node& node, int depth) {
    long total = 0;
    long self = 0;
    int count = 0;
    for (auto&& call : node.calls) {
        count++;
        total += call.elapsed;
        self += call.self;
    }
    out << "<details><summary>";
    out << "<span class='count'>" << count << "</span>";
    out << "<span class='name'>" << node.name << "</span>";
    out << "<span class='time'><span class='self'>" << (self / 1e6) << "</span><span class='total'>" << (total / 1e6) << "</span></span>";
    out << "</summary>";

    for (auto&& next : node.callee) {
        profiling_tree(out, __profiling.calltree[next.second], depth + 1);
    }

    out << "</details>";
}
