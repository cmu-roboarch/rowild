/*
 * Got from <https://github.com/ElliotLockerman/cpp_args>, with minor
 * modifications.
 * */
#pragma once


#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <string>
#include <cstdint>
#include <sstream>
#include <cstring>
#include <cstdio>
#include <utility>
#include <type_traits>
#include <algorithm>

#include "log.h"

namespace args {

////////////////////////////////////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////////////////////////////////////

class StringView {
public:
    StringView() = default;

    StringView(const char* str) : start(str), end(start + strlen(str)) {}
    StringView(const std::string& str) : start(str.c_str()),
        end(start + str.size()) {}


    StringView(const char* str, size_t off, size_t len=npos)
    : start(str + off), end(start + len) {
        if (len == npos) {
            end = str + strlen(str);
        } else {
            assert(len <= strlen(start) - off);
        }
    }

    StringView(const std::string& str, size_t off, size_t len=npos)
    : start(str.c_str()+off), end(start + len) {
        if (len == npos) {
            end = str.c_str() + str.size();
        } else {
            assert(len <= str.size() - off);
        }
    }


    StringView(const StringView&) = default;

    StringView(const StringView sv, size_t off, size_t len=npos) {
        *this = sv.substr(off, len);
    }

    StringView substr(size_t off, size_t len=npos) const {
        assert(start + off + len <= end);
        return StringView(start, off, len);
    }

    size_t size() const { return (size_t)(end - start); }

    bool operator==(const StringView& rhs) const {
        return this->compare(rhs) == 0;
    }

    bool operator!=(const StringView& rhs) const {
        return this->compare(rhs) != 0;
    }

    int compare(const StringView& rhs) const {
        for (size_t i = 0; i <= size(); ++i) {

            // Got to the end of both -> equal
            if (i == this->size() && i == rhs.size()) {
                return 0;

            // lhs finished first -> lhs is greater
            } else if (i == this->size()) {
                return 1;

            // lhs finished first -> rhs is greater
            } else if (i == rhs.size()) {
                return -1;
            }

            auto diff = this->at(i) - rhs[i];
            if (diff != 0) {
                return diff;
            }
        }

        panic("Unreachable");
    }

    // I should write something more general, but this is enough for now...
    size_t find(char ch, size_t off=0) const {
        auto* p = start + off;
        while (p != end) {
            if (*p == ch) { return (size_t)(p - start); }
            ++p;
        }
        return npos;
    }

    char operator[](size_t idx) const {
        assert(start + idx < end);
        return *(start + idx);
    }

    char at(size_t idx) const { return (*this)[idx]; }

    std::string str() const { return std::string(start, end); }


    friend bool operator<(const StringView& lhs, const StringView& rhs) {
        return lhs.compare(rhs) < 0;
    }

    friend std::ostream& operator<<(std::ostream& os, const StringView& sv) {
        auto* p = sv.start;
        while (p != sv.end) {
            os << *p;
            ++p;
        }

        return os;
    }

    static const size_t npos = (size_t)-1;


    // https://stackoverflow.com/a/1449527
    struct ReadBuf : public std::streambuf {
        ReadBuf(const char* s, const char* end) {
            setg((char*)s, (char*)s, (char*)end);
        }
    };

    ReadBuf read_buf() const {
        return ReadBuf(start, end);
    }


private:
    const char* start = nullptr;
    const char* end = nullptr;
};


////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////
class KVArgBase;
class PosArgBase;
class FlagArg;
class VarArgBase;

class ParserBase {
public:
    virtual ~ParserBase() {}
    virtual void add_pos_arg(PosArgBase *pos_arg) = 0;
    virtual void add_kv_arg(KVArgBase *kv_arg) = 0;
    virtual void add_flag_arg(FlagArg *flag_arg) = 0;
    virtual void add_vararg(VarArgBase* vararg) = 0;
};


////////////////////////////////////////////////////////////////////////////////
// Argument classes
////////////////////////////////////////////////////////////////////////////////
class ArgBase {
public:
    ArgBase(const char* _name, const char *_desc)
    : name(_name), desc(_desc) {}

    virtual ~ArgBase() {}

    const char *get_desc() const { return desc; }
    const char *get_name() const { return name; }

    bool found() const { return was_found; }
    operator bool() const { return found(); }

protected:
    bool was_found = false;
    const char *name;
    const char *desc;
};



class PosArgBase : public ArgBase {
public:
    PosArgBase(ParserBase& parser, const char* _name, const char *_desc)
    : ArgBase(_name, _desc) {
        parser.add_pos_arg(this);
    }

    virtual bool parse(StringView str) = 0;
};

template<typename T>
class PosArg : public PosArgBase {
public:
    PosArg(ParserBase& parser, const char* _name, const char *_desc)
    : PosArgBase(parser, _name, _desc) {}


    bool parse(StringView str) override {
        was_found = true;

        auto buf = str.read_buf();
        std::istream is(&buf);

        // Kasraa: This fails for some reasons when it's called from *.cu files
        // assert(is);

        is >> val;
        if (!is) {
            return false;
        }

        is.peek();
        if (!is.eof()) {
            return false;
        }

        return true;
    }


    const T& value() const {
        assert(was_found);
        return val;
    }

    const T& operator*() const {
        return value();
    }

private:
    T val{};
};


class VarArgBase : public ArgBase {
public:
    VarArgBase(ParserBase& parser, const char* _name, const char *_desc)
    : ArgBase(_name, _desc) {
        parser.add_vararg(this);
    }

    virtual bool parse(StringView str) = 0;
};


template<typename T>
class VarArg : public VarArgBase {
    // To prevent confusion with operator bool
    static_assert(!std::is_same<T, bool>::value, "Use FlagArg for bool");

public:
    VarArg(ParserBase& parser, const char* _name, const char *_desc)
    : VarArgBase(parser, _name, _desc) {}

    bool parse(StringView str) override {
        was_found = true;

        auto buf = str.read_buf();
        std::istream is(&buf);

        // Kasraa: This fails for some reasons when it's called from *.cu files
        // assert(is);

        T val{};
        is >> val;
        if (!is) {
            return false;
        }

        is.peek();
        if (!is.eof()) {
            return false;
        }

        vals.push_back(val);
        return true;
    }

    const std::vector<T>& value() const {
        return vals;
    }

    const std::vector<T>& operator*() const {
        return value();
    }

private:
    std::vector<T> vals;
};


class KVArgBase : public ArgBase {
public:
    KVArgBase(ParserBase& parser, const char* _k, const char* _short_k, const char* _desc)
    : ArgBase(_k, _desc), k(_k), short_k(_short_k) {
        parser.add_kv_arg(this);
    }

    virtual bool parse(StringView str) = 0;

    const char* get_key() const { return k; }
    const char* get_short_key() const { return short_k; }

protected:
    const char* k;
    const char* short_k;
};


template<typename T>
class KVArg : public KVArgBase {
    // To prevent confusion with operator bool
    static_assert(!std::is_same<T, bool>::value, "Use FlagArg for bool");
public:
    KVArg(ParserBase& parser, const char* _k, const char* _short_k, const char* _desc)
    : KVArgBase(parser, _k, _short_k, _desc) { }

    bool parse(StringView str) override {
        was_found = true;

        auto buf = str.read_buf();
        std::istream is(&buf);

        is >> val;
        if (!is) {
            return false;
        }

        is.peek();
        if (!is.eof()) {
            return false;
        }

        return true;
    }


    const T& value() const {
        assert(was_found);
        return val;
    }

    const T& value_or(T def) const {
        return was_found ? val : def;
    }

    const T& operator*() const {
        return value();
    }

private:
    T val{};
};



class FlagArg : public ArgBase {
public:
    FlagArg(ParserBase& parser, const char* _k, const char* _short_k, const char *_desc)
    : ArgBase(_k, _desc), k(_k), short_k(_short_k) {
        parser.add_flag_arg(this);
    }

    void parse() {
        was_found = true;
    }

    bool value() const {
        return was_found;
    }

    bool operator*() const {
        return value();
    }

    const char* get_key() const { return k; }
    const char* get_short_key() const { return short_k; }

protected:
    const char* k;
    const char* short_k;
};





////////////////////////////////////////////////////////////////////////////////
// Parser
////////////////////////////////////////////////////////////////////////////////

const char* status_str[] = {
    "SUCCESS",
    "INVALID_KEY",
    "MISSING_VALUE",
    "EXTRA_VALUE",
    "ISTREAM_ERROR",
    "IS_FLAG",
    "MISSING_ARG",
    "EXTRA_ARG",
    "HELP"
};

enum class Status {
    SUCCESS = 0,
    INVALID_KEY,
    MISSING_VALUE,
    EXTRA_VALUE,
    ISTREAM_ERROR,
    IS_FLAG,
    MISSING_ARG,
    EXTRA_ARG,
    HELP
};

static inline std::ostream& operator<<(std::ostream& os, Status s) {
    os << status_str[(int)s];
    return os;
}

struct Result {
    Status status;
    std::string item;

    explicit Result(Status _status, const std::string& _item) : status(_status), item(_item) {}
    explicit Result(Status _status, const char* _item) : status(_status), item(_item) {}
    explicit Result(Status _status, char _item) : status(_status), item(1,_item) {}

    operator bool() { return status == Status::SUCCESS; }
};




class Parser : public ParserBase {
public:
    Parser(const char* _app_name, int argc, const char **argv, bool _silent=false)
    : app_name(_app_name), silent(_silent) {
        for (int i=1; i<argc; i++) { args.emplace_back(argv[i]); }
    }

// Adding arguments
//////////////////////////////////////////////////////////////////////////////
    void add_pos_arg(PosArgBase *pos_arg) override {
        if (vararg) {
            panic("Parser config error: config %s: can't have positional argument after vararg", pos_arg->get_name());
        }
        pos_args.push_back(pos_arg);
    }

    void add_vararg(VarArgBase *_vararg) override {
        if (vararg) {
            panic("Parser config error: config %s: can't have more than one vararg", _vararg->get_name());
        }
        vararg = _vararg;
    }

    void add_kv_arg(KVArgBase *kv_arg) override {
        StringView k = kv_arg->get_key();
        StringView short_k = kv_arg->get_short_key();

        if (k == "help") {
            panic("Parser config error: config %s's key cannot be \"help\" (configs with builtin help flag", kv_arg->get_name());
        }

        if (short_k == "h") {
            panic("Parser config error: config %s's short key cannot be \"h\" (configs with builtin help flag", kv_arg->get_name());
        }

        if (k.size() == 0) {
            panic("Parser config error: config %s's key cannot be empty", kv_arg->get_name());
        }

        if (k.find('=') != StringView::npos) {
            panic("Parser config error: config %s's key cannot contain \"=\"", kv_arg->get_name());
        }

        // kv_keys.push_back(kv_arg);

        if (kv_keys.count(k) != 0 || flag_keys.count(k) != 0) {
            panic("Parser config error: config %s's long key is a duplicate", kv_arg->get_name());
        }
        kv_keys[k] = kv_arg;

        if (short_k != "") {
            if (short_k.size() > 1) {
                panic("Parser config error: config %s's short key %s is %zu characters; A short key must be zero characters (no short key) or one character", kv_arg->get_name(), short_k.str().c_str(), short_k.size());
            }
            char c = short_k[0];
            if (kv_short_keys.count(c) != 0 || flag_short_keys.count(c) != 0) {
                panic("Parser config error: config %s's short key %c is a duplicate", kv_arg->get_name(), c);
            }
            kv_short_keys[c] = kv_arg;
        }

    }


    void add_flag_arg(FlagArg *flag_arg) override {
        StringView k = flag_arg->get_key();
        StringView short_k = flag_arg->get_short_key();

        if (k.size() == 0) {
            panic("Parser config error: config %s's key cannot be empty", flag_arg->get_name());
        }

        // flag_keys.push_back(flag_arg);

        if (flag_keys.count(k) != 0 || kv_keys.count(k) != 0) {
            panic("Parser config error: config %s's key is a duplicate", flag_arg->get_name());
        }
        flag_keys[k] = flag_arg;

        if (short_k != "") {
            if (short_k.size() > 1) {
                panic("Parser config error: config %s's short key %s is %zu characters; A short key must be zero characters (no short key) or one character\n", flag_arg->get_name(), short_k.str().c_str(), short_k.size());
            }
            char c = short_k[0];
            if (flag_short_keys.count(c) != 0 || kv_short_keys.count(c) != 0) {
                panic("Parser config error: config %s's short key %c is a duplicate", flag_arg->get_name(), c);
            }
            flag_short_keys[c] = flag_arg;
        }

    }



// Parsing arguments
//////////////////////////////////////////////////////////////////////////////
    Result parse() {

        std::reverse(args.begin(), args.end());

        while (!args.empty()) {
            // Parse functions pop, so arg no longer valid after call
            auto& arg = args.back();

            if (!saw_double_dash && arg == "--") {
                args.pop_back();
                saw_double_dash = true;
                continue;

            // Long key
            } else if (!saw_double_dash && arg.size() > 2 && arg.substr(0, 2) == "--") {
                auto res = parse_long_arg();
                if (!res) {
                    return res;
                }


             // Short key
            } else if (!saw_double_dash && arg.size() > 1 && arg[0] == '-') {
                auto res = parse_short_arg();
                if (!res) {
                    return res;
                }

            // Positional arg
            } else {
                if (pos_arg_idx < pos_args.size()) {
                    auto res = parse_positional_arg();
                    if (!res) {
                        return res;
                    }
                } else if (vararg) {
                    auto res = parse_vararg();
                    if (!res) {
                        return res;
                    }
                } else {
                    // Extranous positional arg
                    if (!silent) {
                        fprintf(stderr, "Too many positional arguments\n");
                        print_usage();
                    }
                    return Result(Status::EXTRA_ARG, "");
                }
            }
        }




        if (pos_arg_idx < pos_args.size()) {
            if (!silent) {
                fprintf(stderr, "Missing required positional argument(s)\n");
                print_usage();
            }
            return Result(Status::MISSING_ARG, "");
        }

        return Result(Status::SUCCESS, "");
    }

    Result parse_long_arg() {
        auto arg = args.back();
        args.pop_back();

        auto eq = arg.find('=');
        StringView key;
        StringView value;

        // Get key;
        if (eq != StringView::npos) {
            key = arg.substr(2, eq - key.size() - 2);
        } else {
            key = arg.substr(2, StringView::npos);
        }

        if (key == "help") {
            print_usage();
            return Result(Status::HELP, "");
        }

        auto it = kv_keys.find(key);
        if (it == kv_keys.end()) {
            auto it2 = flag_keys.find(key);
            if (it2 == flag_keys.end()) {
                if (!silent) {
                    fprintf(stderr, "Long argument key --%s invalid\n", key.str().c_str());
                    print_usage();
                }
                return Result(Status::INVALID_KEY, key.str());
            }

            it2->second->parse();
            return Result(Status::SUCCESS, "");
        }


        // Get value
        if (eq != StringView::npos) {
            value = arg.substr(eq+1, StringView::npos);
        } else {
            if (args.empty()) {
                if (!silent) {
                    fprintf(stderr, "Long argument key --%s needs value\n", key.str().c_str());
                    print_usage();
                }
                return Result(Status::MISSING_VALUE, key.str());
            }

            value = std::move(args.back());
            args.pop_back();
        }

        bool good = it->second->parse(value);
        if (!good) {
            if (!silent) {
                fprintf(stderr, "Could not parse value of argument --%s\n", key.str().c_str());
                print_usage();
            }
            return Result(Status::ISTREAM_ERROR, key.str());
        }


        return Result(Status::SUCCESS, "");
    }


    Result parse_short_arg() {
        auto arg = args.back();
        args.pop_back();

        char key = arg[1];

        if (key == 'h') {
            print_usage();
            return Result(Status::HELP, "");
        }

        auto it = kv_short_keys.find(key);
        if (it == kv_short_keys.end()) {
            auto it2 = flag_short_keys.find(key);
            if (it2 == flag_short_keys.end()) {
                if (!silent) {
                    fprintf(stderr, "Short argument key -%c invalid\n", key);
                print_usage(); }
                return Result(Status::INVALID_KEY, key);
            } else if (arg.size() > 2) {
                if (!silent) {
                    fprintf(stderr, "Flag -%c doesn't take a value\n", key);
                    print_usage();
                }
                return Result(Status::EXTRA_VALUE, key);
            }

            it2->second->parse();
            return Result(Status::SUCCESS, "");
        }



        StringView value;
        if (arg.size() > 2) {
            value = arg.substr(2, StringView::npos);
        } else {
            if (args.empty()) {
                if (!silent) {
                    fprintf(stderr, "Short argument key -%c needs value\n", key);
                    print_usage();
                }
                return Result(Status::MISSING_VALUE, key);
            }
            value = args.back();
            args.pop_back();
        }


        bool good = it->second->parse(value);
        if (!good) {
            if (!silent) {
                fprintf(stderr, "Could not parse value of argument -%c\n", key);
                print_usage();
            }
            return Result(Status::ISTREAM_ERROR, key);
        }

        return Result(Status::SUCCESS, "");
    }

    Result parse_positional_arg() {
        auto arg = args.back();
        args.pop_back();

        assert(pos_arg_idx < pos_args.size());
        auto& pos_arg = pos_args.at(pos_arg_idx);
        bool good = pos_arg->parse(arg);
        if (!good) {
            if (!silent) {
                fprintf(stderr, "Could not parse positional argument \"%s\"\n", arg.str().c_str());
                print_usage();
            }
            return Result(Status::ISTREAM_ERROR, pos_arg->get_name());
        }
        pos_arg_idx++;
        return Result(Status::SUCCESS, "");
    }

    Result parse_vararg() {
        auto arg = args.back();
        args.pop_back();

        bool good = vararg->parse(arg);
        if (!good) {
            if (!silent) {
                fprintf(stderr, "Could not parse vararg \"%s\"\n", arg.str().c_str());
                print_usage();
            }
            return Result(Status::ISTREAM_ERROR, vararg->get_name());
        }
        return Result(Status::SUCCESS, "");
    }

    void print_usage() const {
        fprintf(stderr, "USAGE:\n");
        fprintf(stderr, "\t%s ", app_name);

        if (kv_keys.size() > 0) {
            fprintf(stderr, " [OPTIONS] ");
        }

        fprintf(stderr, "[FLAGS] ");

        for (auto& config : pos_args) {
            fprintf(stderr, "<%s> ", config->get_name());
        }

        if (vararg) {
            fprintf(stderr, "[%s]...", vararg->get_name());
        }


        fprintf(stderr, "\n");

        if (pos_args.size() > 0) {
            fprintf(stderr, "\nARGS:\n");
            for (auto& config : pos_args) {
                fprintf(stderr, "\t%s\t%s\n", config->get_name(), config->get_desc());
            }
        }

        if (vararg) {
            fprintf(stderr, "\t%s\t%s\n", vararg->get_name(), vararg->get_desc());
        }

        if (kv_keys.size() > 0) {
            fprintf(stderr, "\nOPTIONS:\n");
            for (auto& p : kv_keys) {
                fprintf(stderr, "\t--%s", p.first.str().c_str());

                if (*p.second->get_short_key() != '\0') {
                    fprintf(stderr, ", -%s", p.second->get_short_key());
                }

                fprintf(stderr, " <val>\t%s\n", p.second->get_desc());
            }
        }


        fprintf(stderr, "\nFLAGS:\n");
        for (auto& p : flag_keys) {
            fprintf(stderr, "\t--%s", p.first.str().c_str());

            if (*p.second->get_short_key() != '\0') {
                fprintf(stderr, ", -%s", p.second->get_short_key());
            }

            fprintf(stderr, "\t%s\n", p.second->get_desc());
        }
        fprintf(stderr, "\t--help, -h\tPrint help message\n");
    }


private:
    const char* app_name;
    std::vector<StringView> args;
    bool silent = false;

    uint32_t pos_arg_idx = 0;
    std::vector<PosArgBase*> pos_args;

    std::map<StringView, KVArgBase*> kv_keys;
    std::map<char, KVArgBase*> kv_short_keys;

    std::map<StringView, FlagArg*> flag_keys;
    std::map<char, FlagArg*> flag_short_keys;

    VarArgBase* vararg = nullptr;

    bool saw_double_dash = false;
};



} // namespace parser
