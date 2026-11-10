// Command parser: peekCommandType, every error code and field, all yaw
// variants, cfg patches (atomicity, preset, cross-field validation).
#include <string.h>
#include <unity.h>

#include <string>

#include "cmd_parser.h"

using proto::Cmd;
using proto::Err;
using proto::YawAction;
using proto::YawDir;

void setUp() {}
void tearDown() {}

static ParseResult parse(const char* json, Command& c, const Settings* base = nullptr,
                         Settings* out = nullptr) {
  return parseCommand(json, strlen(json), c, base, out);
}

static Cmd peek(const char* json) { return peekCommandType(json, strlen(json)); }

#define ASSERT_OK(r) TEST_ASSERT_EQUAL_INT_MESSAGE((int)Err::None, (int)(r).err, "expected success")
#define ASSERT_ERR(r, e, f)                                  \
  do {                                                       \
    TEST_ASSERT_EQUAL_INT((int)(e), (int)(r).err);           \
    if ((f) == nullptr) {                                    \
      TEST_ASSERT_NULL((r).field);                           \
    } else {                                                 \
      TEST_ASSERT_NOT_NULL((r).field);                       \
      TEST_ASSERT_EQUAL_STRING((const char*)(f), (r).field); \
    }                                                        \
  } while (0)

static const char* const kNoField = nullptr;

// ---------------------------------------------------------------------------
// peekCommandType
// ---------------------------------------------------------------------------

static void test_peek_basic() {
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)peek("{\"t\":\"stop\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Rotor, (int)peek("{\"t\":\"rotor\",\"speed\":5}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Yaw, (int)peek("{\"t\":\"yaw\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)peek("{\"t\":\"ping\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Cfg, (int)peek("{\"t\":\"cfg\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Info, (int)peek("{\"t\":\"info\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Log, (int)peek("{\"t\":\"log\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Reboot, (int)peek("{\"t\":\"reboot\"}"));
}

static void test_peek_whitespace_and_position() {
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)peek("  \r\n{ \"t\" :\t\"stop\" }"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)peek("{\"id\":5,\"t\":\"stop\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)peek("{\"a\":\"x\" , \"t\":\"stop\"}"));
}

static void test_peek_only_top_level_t() {
  TEST_ASSERT_EQUAL_INT((int)Cmd::Rotor,
                        (int)peek("{\"x\":{\"t\":\"stop\"},\"t\":\"rotor\",\"speed\":5}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Yaw, (int)peek("{\"a\":[1,{\"t\":\"stop\"}],\"t\":\"yaw\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"x\":{\"t\":\"stop\"}}"));
  // "t":"stop" inside a string value
  TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)peek("{\"m\":\"\\\"t\\\":\\\"stop\\\"\",\"t\":\"ping\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"m\":\"{\\\"t\\\":\\\"stop\\\"}\"}"));
  // "t" used as a value, not a key
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"k\":\"t\",\"v\":\"stop\"}"));
  // top level must be an object
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("[{\"t\":\"stop\"}]"));
}

static void test_peek_exact_value_only() {
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\"stopx\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\"sto\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\"STOP\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\" stop\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"tt\":\"stop\"}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":5}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":[\"stop\"]}"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\"st\\u006fp\"}"));  // escaped: cannot tell
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"\\u0074\":\"stop\"}"));   // escaped key
}

static void test_peek_duplicate_last_wins_like_parser() {
  const char* a = "{\"t\":\"stop\",\"t\":\"ping\"}";
  const char* b = "{\"t\":\"ping\",\"t\":\"stop\"}";
  Command c;
  TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)peek(a));
  ASSERT_OK(parse(a, c));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)c.type);
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)peek(b));
  ASSERT_OK(parse(b, c));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)c.type);
  // a later non-string "t" overrides an earlier "stop"
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\"stop\",\"t\":1}"));
}

static void test_peek_garbage_and_bounds() {
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peekCommandType(nullptr, 10));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek(""));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("stop"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\""));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\":\"sto"));
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peek("{\"t\" \"stop\"}"));
  // len is honoured: the value is cut off
  const char* s = "{\"t\":\"stop\"}";
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peekCommandType(s, 8));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)peekCommandType(s, 11));  // closing brace cut
  // garbage after the object is not looked at
  TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)peek("{\"t\":\"ping\"}{\"t\":\"stop\"}"));
  // a buffer with no NUL in range must not be overrun
  char raw[4] = {'{', '"', 't', '"'};
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)peekCommandType(raw, sizeof(raw)));
}

static void test_peek_agrees_with_parser() {
  static const char* const kFrames[] = {
      "{\"t\":\"stop\"}",
      "{\"t\":\"rotor\",\"speed\":10}",
      "{\"id\":1,\"t\":\"yaw\",\"dir\":\"left\"}",
      "{\"x\":{\"t\":\"stop\"},\"t\":\"ping\"}",
      "{\"m\":\"t\",\"t\":\"cfg\"}",
      " {\"t\" : \"info\"} ",
      "{\"t\":\"log\",\"a\":[{\"t\":\"stop\"}]}",
  };
  for (const char* f : kFrames) {
    Command c;
    ASSERT_OK(parse(f, c));
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)c.type, (int)peek(f), f);
  }
}

// ---------------------------------------------------------------------------
// Framing errors
// ---------------------------------------------------------------------------

static std::string padded(const char* head, size_t total) {
  // {"t":"ping","pad":"xxx..."} of exactly `total` bytes
  std::string s = std::string("{\"t\":\"") + head + "\",\"pad\":\"";
  while (s.size() < total - 2) s += 'x';
  s += "\"}";
  return s;
}

static void test_too_long() {
  std::string ok = padded("ping", proto::kMaxCmdLen);
  TEST_ASSERT_EQUAL_UINT32(256, ok.size());
  Command c;
  ASSERT_OK(parseCommand(ok.c_str(), ok.size(), c));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)c.type);

  std::string tooLong = padded("cfg", proto::kMaxCmdLen + 1);
  ParseResult r = parseCommand(tooLong.c_str(), tooLong.size(), c);
  ASSERT_ERR(r, Err::TooLong, kNoField);
  TEST_ASSERT_EQUAL_INT((int)Cmd::Cfg, (int)c.type);  // "of" from the cheap scan
}

static void test_bad_json() {
  static const char* const kBad[] = {"",        "not json", "[1,2]",         "\"str\"", "42",
                                     "null",    "{",        "{\"t\":\"ping\"", "{\"t\":}", "{\"t\":\"ping\",}",
                                     "{\"t\" \"ping\"}"};
  for (const char* s : kBad) {
    Command c;
    ParseResult r = parse(s, c);
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)Err::BadJson, (int)r.err, s);
    TEST_ASSERT_NULL(r.field);
    TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)c.type);
  }
  Command c;
  ASSERT_ERR(parseCommand(nullptr, 0, c), Err::BadJson, kNoField);
}

static void test_bad_t() {
  Command c;
  ASSERT_ERR(parse("{}", c), Err::BadType, "t");
  ASSERT_ERR(parse("{\"t\":5}", c), Err::BadType, "t");
  ASSERT_ERR(parse("{\"t\":null}", c), Err::BadType, "t");
  ASSERT_ERR(parse("{\"t\":{\"a\":1}}", c), Err::BadType, "t");
  ASSERT_ERR(parse("{\"type\":\"ping\"}", c), Err::BadType, "t");
  ASSERT_ERR(parse("{t:1}", c), Err::BadType, "t");  // ArduinoJson accepts bare keys
}

static void test_unknown_cmd() {
  Command c;
  ParseResult r = parse("{\"t\":\"fly\",\"id\":9}", c);
  ASSERT_ERR(r, Err::UnknownCmd, kNoField);
  TEST_ASSERT_EQUAL_INT((int)Cmd::None, (int)c.type);
  TEST_ASSERT_TRUE(c.hasId);  // still echoed
  TEST_ASSERT_EQUAL_UINT32(9, c.id);
  ASSERT_ERR(parse("{\"t\":\"\"}", c), Err::UnknownCmd, kNoField);
  ASSERT_ERR(parse("{\"t\":\"Stop\"}", c), Err::UnknownCmd, kNoField);
}

static void test_simple_commands() {
  struct {
    const char* json;
    Cmd type;
  } cases[] = {{"{\"t\":\"stop\"}", Cmd::Stop},     {"{\"t\":\"ping\"}", Cmd::Ping},
               {"{\"t\":\"info\"}", Cmd::Info},     {"{\"t\":\"log\"}", Cmd::Log},
               {"{\"t\":\"reboot\"}", Cmd::Reboot}, {"{\"t\":\"ping\",\"x\":[1,{}]}", Cmd::Ping}};
  for (auto& k : cases) {
    Command c;
    ASSERT_OK(parse(k.json, c));
    TEST_ASSERT_EQUAL_INT((int)k.type, (int)c.type);
    TEST_ASSERT_FALSE(c.hasId);
  }
}

// ---------------------------------------------------------------------------
// id
// ---------------------------------------------------------------------------

static void test_id() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"ping\",\"id\":17}", c));
  TEST_ASSERT_TRUE(c.hasId);
  TEST_ASSERT_EQUAL_UINT32(17, c.id);
  ASSERT_OK(parse("{\"t\":\"ping\",\"id\":0}", c));
  TEST_ASSERT_TRUE(c.hasId);
  TEST_ASSERT_EQUAL_UINT32(0, c.id);
  ASSERT_OK(parse("{\"t\":\"ping\",\"id\":4294967295}", c));
  TEST_ASSERT_EQUAL_UINT32(4294967295u, c.id);
  ASSERT_OK(parse("{\"t\":\"ping\",\"id\":null}", c));
  TEST_ASSERT_FALSE(c.hasId);

  static const char* const kBad[] = {
      "{\"t\":\"ping\",\"id\":4294967296}", "{\"t\":\"ping\",\"id\":-1}",
      "{\"t\":\"ping\",\"id\":1.5}",        "{\"t\":\"ping\",\"id\":\"5\"}",
      "{\"t\":\"ping\",\"id\":true}",       "{\"t\":\"ping\",\"id\":[1]}",
      "{\"t\":\"ping\",\"id\":1e3}",
  };
  for (const char* s : kBad) {
    ParseResult r = parse(s, c);
    TEST_ASSERT_EQUAL_INT_MESSAGE((int)Err::BadType, (int)r.err, s);
    TEST_ASSERT_EQUAL_STRING("id", r.field);
    TEST_ASSERT_EQUAL_INT((int)Cmd::Ping, (int)c.type);
    TEST_ASSERT_FALSE(c.hasId);
  }
}

static void test_id_kept_on_field_error() {
  Command c;
  ParseResult r = parse("{\"t\":\"rotor\",\"id\":3}", c);
  ASSERT_ERR(r, Err::BadType, "speed");
  TEST_ASSERT_EQUAL_INT((int)Cmd::Rotor, (int)c.type);
  TEST_ASSERT_TRUE(c.hasId);
  TEST_ASSERT_EQUAL_UINT32(3, c.id);
}

static void test_seq() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"rotor\",\"speed\":30,\"seq\":4294967295}", c));
  TEST_ASSERT_TRUE(c.hasSeq);
  TEST_ASSERT_EQUAL_UINT32(4294967295u, c.seq);
  TEST_ASSERT_FALSE(c.hasId);
  ASSERT_OK(parse("{\"t\":\"stop\",\"seq\":17,\"id\":3}", c));
  TEST_ASSERT_TRUE(c.hasSeq);
  TEST_ASSERT_EQUAL_UINT32(17u, c.seq);
  TEST_ASSERT_EQUAL_UINT32(3u, c.id);
  ASSERT_OK(parse("{\"t\":\"ping\",\"seq\":null}", c));
  TEST_ASSERT_FALSE(c.hasSeq);
  ASSERT_OK(parse("{\"t\":\"ping\"}", c));
  TEST_ASSERT_FALSE(c.hasSeq);
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":3,\"seq\":-1}", c), Err::BadType, "seq");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":3,\"seq\":4294967296}", c), Err::BadType, "seq");
  ASSERT_ERR(parse("{\"t\":\"stop\",\"seq\":\"5\"}", c), Err::BadType, "seq");
  ASSERT_ERR(parse("{\"t\":\"stop\",\"seq\":1.5}", c), Err::BadType, "seq");
  TEST_ASSERT_EQUAL_INT((int)Cmd::Stop, (int)c.type);  // still known for the "of" field
}

static void test_command_is_reset() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"rotor\",\"speed\":50,\"brake\":true,\"id\":4}", c));
  ASSERT_OK(parse("{\"t\":\"ping\"}", c));
  TEST_ASSERT_FALSE(c.hasId);
  TEST_ASSERT_EQUAL_UINT8(0, c.speed);
  TEST_ASSERT_FALSE(c.brake);
}

// ---------------------------------------------------------------------------
// rotor
// ---------------------------------------------------------------------------

static void test_rotor_speed() {
  struct {
    const char* json;
    uint8_t speed;
  } ok[] = {{"{\"t\":\"rotor\",\"speed\":55}", 55},   {"{\"t\":\"rotor\",\"speed\":0}", 0},
            {"{\"t\":\"rotor\",\"speed\":100}", 100}, {"{\"t\":\"rotor\",\"speed\":55.4}", 55},
            {"{\"t\":\"rotor\",\"speed\":55.5}", 56}, {"{\"t\":\"rotor\",\"speed\":100.4}", 100},
            {"{\"t\":\"rotor\",\"speed\":-0.4}", 0},  {"{\"t\":\"rotor\",\"speed\":1e1}", 10}};
  for (auto& k : ok) {
    Command c;
    ASSERT_OK(parse(k.json, c));
    TEST_ASSERT_EQUAL_INT((int)Cmd::Rotor, (int)c.type);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(k.speed, c.speed, k.json);
    TEST_ASSERT_FALSE(c.brake);
  }
}

static void test_rotor_errors() {
  Command c;
  ASSERT_ERR(parse("{\"t\":\"rotor\"}", c), Err::BadType, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":null}", c), Err::BadType, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":\"50\"}", c), Err::BadType, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":true}", c), Err::BadType, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":[50]}", c), Err::BadType, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":101}", c), Err::OutOfRange, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":100.6}", c), Err::OutOfRange, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":-1}", c), Err::OutOfRange, "speed");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":1e9}", c), Err::OutOfRange, "speed");
  // ArduinoJson rejects NaN / Infinity tokens (ARDUINOJSON_ENABLE_NAN = 0)
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":NaN}", c), Err::BadJson, kNoField);
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":-Infinity}", c), Err::BadJson, kNoField);
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":0,\"brake\":\"yes\"}", c), Err::BadType, "brake");
  ASSERT_ERR(parse("{\"t\":\"rotor\",\"speed\":0,\"brake\":1}", c), Err::BadType, "brake");
  TEST_ASSERT_EQUAL_INT((int)Cmd::Rotor, (int)c.type);
}

static void test_rotor_brake() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"rotor\",\"speed\":0,\"brake\":true}", c));
  TEST_ASSERT_TRUE(c.brake);
  TEST_ASSERT_EQUAL_UINT8(0, c.speed);
  ASSERT_OK(parse("{\"t\":\"rotor\",\"speed\":0,\"brake\":false}", c));
  TEST_ASSERT_FALSE(c.brake);
  ASSERT_OK(parse("{\"t\":\"rotor\",\"speed\":0,\"brake\":null}", c));
  TEST_ASSERT_FALSE(c.brake);
}

// ---------------------------------------------------------------------------
// yaw
// ---------------------------------------------------------------------------

static void test_yaw_dir() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"left\"}", c));
  TEST_ASSERT_EQUAL_INT((int)Cmd::Yaw, (int)c.type);
  TEST_ASSERT_EQUAL_INT((int)YawAction::Move, (int)c.yawAction);
  TEST_ASSERT_EQUAL_INT((int)YawDir::Left, (int)c.dir);
  TEST_ASSERT_EQUAL_UINT8(0, c.yawSpeed);  // use the setting

  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"right\",\"speed\":60}", c));
  TEST_ASSERT_EQUAL_INT((int)YawDir::Right, (int)c.dir);
  TEST_ASSERT_EQUAL_UINT8(60, c.yawSpeed);

  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"right\",\"speed\":19.6}", c));
  TEST_ASSERT_EQUAL_UINT8(20, c.yawSpeed);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"speed\":100}", c));
  TEST_ASSERT_EQUAL_UINT8(100, c.yawSpeed);

  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"stop\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::Move, (int)c.yawAction);
  TEST_ASSERT_EQUAL_INT((int)YawDir::Stop, (int)c.dir);
  // a stop never fails because of an optional field
  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"stop\",\"speed\":0}", c));
  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"stop\",\"speed\":\"x\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawDir::Stop, (int)c.dir);
}

static void test_yaw_dir_errors() {
  Command c;
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"up\"}", c), Err::OutOfRange, "dir");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"LEFT\"}", c), Err::OutOfRange, "dir");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":5}", c), Err::BadType, "dir");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":true}", c), Err::BadType, "dir");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"speed\":19}", c), Err::OutOfRange, "speed");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"speed\":101}", c), Err::OutOfRange, "speed");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"speed\":0}", c), Err::OutOfRange, "speed");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"speed\":\"60\"}", c), Err::BadType, "speed");
  TEST_ASSERT_EQUAL_INT((int)Cmd::Yaw, (int)c.type);
}

static void test_yaw_hold() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"yaw\",\"hold\":\"left\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::Hold, (int)c.yawAction);
  TEST_ASSERT_EQUAL_INT((int)YawDir::Left, (int)c.dir);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"hold\":\"right\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawDir::Right, (int)c.dir);
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"hold\":\"stop\"}", c), Err::OutOfRange, "hold");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"hold\":1}", c), Err::BadType, "hold");
}

static void test_yaw_goto() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"yaw\",\"goto\":135}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::Goto, (int)c.yawAction);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 135.0f, c.gotoDeg);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"goto\":0}", c));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, c.gotoDeg);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"goto\":359.9}", c));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 359.9f, c.gotoDeg);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"goto\":359.999999999}", c));
  TEST_ASSERT_TRUE(c.gotoDeg >= 0.0f && c.gotoDeg < 360.0f);
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"goto\":360}", c), Err::OutOfRange, "goto");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"goto\":-1}", c), Err::OutOfRange, "goto");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"goto\":\"90\"}", c), Err::BadType, "goto");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"goto\":false}", c), Err::BadType, "goto");
}

static void test_yaw_zero_and_cal() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"yaw\",\"zero\":true}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::Zero, (int)c.yawAction);
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"zero\":false}", c), Err::OutOfRange, "zero");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"zero\":1}", c), Err::BadType, "zero");

  ASSERT_OK(parse("{\"t\":\"yaw\",\"cal\":\"start\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::CalStart, (int)c.yawAction);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"cal\":\"stop\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::CalStop, (int)c.yawAction);
  ASSERT_OK(parse("{\"t\":\"yaw\",\"cal\":\"cancel\"}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::CalCancel, (int)c.yawAction);
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"cal\":\"go\"}", c), Err::OutOfRange, "cal");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"cal\":true}", c), Err::BadType, "cal");
}

static void test_yaw_exactly_one_action() {
  Command c;
  ASSERT_ERR(parse("{\"t\":\"yaw\"}", c), Err::BadType, "dir");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"speed\":50}", c), Err::BadType, "dir");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"goto\":90}", c), Err::BadType, "goto");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"hold\":\"left\",\"zero\":true}", c), Err::BadType, "zero");
  ASSERT_ERR(parse("{\"t\":\"yaw\",\"cal\":\"start\",\"dir\":\"stop\"}", c), Err::BadType, "cal");
  // null counts as absent
  ASSERT_OK(parse("{\"t\":\"yaw\",\"dir\":\"left\",\"goto\":null}", c));
  TEST_ASSERT_EQUAL_INT((int)YawAction::Move, (int)c.yawAction);
}

// ---------------------------------------------------------------------------
// cfg
// ---------------------------------------------------------------------------

static Settings defaults() {
  Settings s;
  settingsDefaults(s);
  return s;
}

static uint64_t bitOf(const char* key) {
  size_t i = 0;
  TEST_ASSERT_NOT_NULL(settingsFindField(key, &i));
  return (uint64_t)1 << i;
}

static void test_cfg_get_and_reset() {
  Command c;
  ASSERT_OK(parse("{\"t\":\"cfg\"}", c));
  TEST_ASSERT_TRUE(c.cfgGet);
  TEST_ASSERT_FALSE(c.cfgReset);
  TEST_ASSERT_EQUAL_UINT64(0, c.cfgMask);
  ASSERT_OK(parse("{\"t\":\"cfg\",\"get\":true}", c));
  TEST_ASSERT_TRUE(c.cfgGet);
  ASSERT_OK(parse("{\"t\":\"cfg\",\"get\":false}", c));
  TEST_ASSERT_TRUE(c.cfgGet);  // nothing to set: still a read
  ASSERT_OK(parse("{\"t\":\"cfg\",\"foo\":1,\"bar\":\"x\"}", c));
  TEST_ASSERT_TRUE(c.cfgGet);  // unknown keys ignored
  ASSERT_OK(parse("{\"t\":\"cfg\",\"get\":true,\"rotorMax\":80}", c));
  TEST_ASSERT_TRUE(c.cfgGet);
  TEST_ASSERT_EQUAL_UINT64(0, c.cfgMask);
  ASSERT_OK(parse("{\"t\":\"cfg\",\"reset\":true}", c));
  TEST_ASSERT_TRUE(c.cfgReset);
  TEST_ASSERT_FALSE(c.cfgGet);
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"get\":\"yes\"}", c), Err::BadType, "get");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"reset\":1}", c), Err::BadType, "reset");
  TEST_ASSERT_EQUAL_INT((int)Cmd::Cfg, (int)c.type);
}

static void test_cfg_set_applies_patch() {
  Settings base = defaults();
  Settings out = defaults();
  out.rotorMax = 1;  // must be overwritten from base
  Command c;
  ASSERT_OK(parse("{\"t\":\"cfg\",\"yawSpeed\":70,\"rotorMax\":80,\"unknown\":5}", c, &base, &out));
  TEST_ASSERT_FALSE(c.cfgGet);
  TEST_ASSERT_FALSE(c.cfgReset);
  TEST_ASSERT_EQUAL_UINT64(bitOf("yawSpeed") | bitOf("rotorMax"), c.cfgMask);
  TEST_ASSERT_EQUAL_UINT8(70, out.yawSpeed);
  TEST_ASSERT_EQUAL_UINT8(80, out.rotorMax);
  TEST_ASSERT_EQUAL_UINT8(base.rotorMin, out.rotorMin);
  TEST_ASSERT_EQUAL_UINT8(60, base.yawSpeed);  // base untouched

  ASSERT_OK(parse("{\"t\":\"cfg\",\"led\":false,\"batCal\":1.023,\"rotorMax\":80.0}", c, &base, &out));
  TEST_ASSERT_FALSE(out.led);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.023f, out.batCal);
  TEST_ASSERT_EQUAL_UINT8(80, out.rotorMax);
  ASSERT_OK(parse("{\"t\":\"cfg\",\"yawMaxBal\":600000,\"name\":\"Wiatrak 1\"}", c, &base, &out));
  TEST_ASSERT_EQUAL_UINT32(600000, out.yawMaxBal);
  TEST_ASSERT_EQUAL_STRING("Wiatrak 1", out.name);
}

static void test_cfg_patch_is_atomic() {
  Settings base = defaults();
  Settings out = defaults();
  Settings before = out;
  Command c;
  // valid first key, bad second key: nothing changes
  ParseResult r = parse("{\"t\":\"cfg\",\"rotorMax\":80,\"yawSpeed\":10}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "yawSpeed");
  TEST_ASSERT_EQUAL_MEMORY(&before, &out, sizeof(Settings));
  TEST_ASSERT_EQUAL_UINT64(0, c.cfgMask);

  r = parse("{\"t\":\"cfg\",\"rotorMax\":\"80\"}", c, &base, &out);
  ASSERT_ERR(r, Err::BadType, "rotorMax");
  r = parse("{\"t\":\"cfg\",\"led\":1}", c, &base, &out);
  ASSERT_ERR(r, Err::BadType, "led");
  r = parse("{\"t\":\"cfg\",\"rotorMax\":80.5}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "rotorMax");
  r = parse("{\"t\":\"cfg\",\"batCal\":2}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "batCal");
  r = parse("{\"t\":\"cfg\",\"name\":5}", c, &base, &out);
  ASSERT_ERR(r, Err::BadType, "name");
  r = parse("{\"t\":\"cfg\",\"name\":\"a\\\"b\"}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "name");
  r = parse("{\"t\":\"cfg\",\"name\":\"0123456789012345678901234\"}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "name");
  r = parse("{\"t\":\"cfg\",\"apChan\":14}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "apChan");
  r = parse("{\"t\":\"cfg\",\"fsTimeout\":300}", c, &base, &out);
  ASSERT_ERR(r, Err::OutOfRange, "fsTimeout");
  TEST_ASSERT_EQUAL_MEMORY(&before, &out, sizeof(Settings));
}

static void test_cfg_in_place_patch() {
  Settings s = defaults();
  Command c;
  ASSERT_OK(parse("{\"t\":\"cfg\",\"rotorMin\":12}", c, &s, &s));
  TEST_ASSERT_EQUAL_UINT8(12, s.rotorMin);
  Settings before = s;
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"rotorMin\":20,\"rotorDuty\":10}", c, &s, &s), Err::OutOfRange,
             "rotorDuty");
  TEST_ASSERT_EQUAL_MEMORY(&before, &s, sizeof(Settings));
}

static void test_cfg_battery_preset() {
  Settings base = defaults();  // 9V preset
  Settings out;
  Command c;
  ASSERT_OK(parse("{\"t\":\"cfg\",\"batType\":1}", c, &base, &out));
  TEST_ASSERT_EQUAL_UINT8(1, out.batType);
  TEST_ASSERT_EQUAL_UINT16(6600, out.batLow);
  TEST_ASSERT_EQUAL_UINT16(6000, out.batCrit);
  TEST_ASSERT_EQUAL_UINT16(6000, out.batEmpty);
  TEST_ASSERT_EQUAL_UINT16(8400, out.batFull);
  TEST_ASSERT_EQUAL_UINT64(bitOf("batType"), c.cfgMask);

  // explicit thresholds in the same patch win over the preset
  ASSERT_OK(parse("{\"t\":\"cfg\",\"batLow\":7000,\"batType\":1}", c, &base, &out));
  TEST_ASSERT_EQUAL_UINT16(7000, out.batLow);
  TEST_ASSERT_EQUAL_UINT16(6000, out.batCrit);

  // custom keeps the current thresholds
  Settings custom = out;
  ASSERT_OK(parse("{\"t\":\"cfg\",\"batType\":2,\"batFull\":8000}", c, &custom, &out));
  TEST_ASSERT_EQUAL_UINT8(2, out.batType);
  TEST_ASSERT_EQUAL_UINT16(7000, out.batLow);
  TEST_ASSERT_EQUAL_UINT16(8000, out.batFull);

  // back to 9V restores its thresholds
  ASSERT_OK(parse("{\"t\":\"cfg\",\"batType\":0}", c, &out, &out));
  TEST_ASSERT_EQUAL_UINT16(7000, out.batLow);
  TEST_ASSERT_EQUAL_UINT16(6200, out.batCrit);
  TEST_ASSERT_EQUAL_UINT16(9400, out.batFull);

  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batType\":3}", c, &base, &out), Err::OutOfRange, "batType");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batType\":\"2S\"}", c, &base, &out), Err::BadType, "batType");
}

static void test_cfg_cross_field_validation() {
  Settings base = defaults();  // low 7000, crit 6200, empty 6200, full 9400
  Settings out = base;
  Settings before = out;
  Command c;
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batCrit\":7500}", c, &base, &out), Err::OutOfRange, "batCrit");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batCrit\":7000}", c, &base, &out), Err::OutOfRange, "batCrit");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batLow\":6000}", c, &base, &out), Err::OutOfRange, "batCrit");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batEmpty\":9500}", c, &base, &out), Err::OutOfRange, "batFull");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batFull\":6200}", c, &base, &out), Err::OutOfRange, "batFull");
  // a preset that conflicts with an explicit value in the same patch
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batType\":1,\"batLow\":5000}", c, &base, &out), Err::OutOfRange,
             "batCrit");
  TEST_ASSERT_EQUAL_MEMORY(&before, &out, sizeof(Settings));
  // moving both together is fine
  ASSERT_OK(parse("{\"t\":\"cfg\",\"batLow\":8000,\"batCrit\":7500}", c, &base, &out));
  TEST_ASSERT_EQUAL_UINT16(8000, out.batLow);
  TEST_ASSERT_EQUAL_UINT16(7500, out.batCrit);
}

static void test_cfg_without_base_only_checks() {
  Command c;
  // no cross-field check without a base
  ASSERT_OK(parse("{\"t\":\"cfg\",\"batCrit\":7500}", c));
  TEST_ASSERT_EQUAL_UINT64(bitOf("batCrit"), c.cfgMask);
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"rotorMax\":5}", c), Err::OutOfRange, "rotorMax");
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"captive\":\"on\"}", c), Err::BadType, "captive");

  // cfgOut without cfgBase is never written
  Settings out = defaults();
  Settings before = out;
  ASSERT_OK(parse("{\"t\":\"cfg\",\"rotorMax\":50}", c, nullptr, &out));
  TEST_ASSERT_EQUAL_MEMORY(&before, &out, sizeof(Settings));
  // cfgBase without cfgOut: fully validated, nothing written
  Settings base = defaults();
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"batCrit\":7500}", c, &base, nullptr), Err::OutOfRange, "batCrit");
  ASSERT_OK(parse("{\"t\":\"cfg\",\"rotorMax\":50}", c, &base, nullptr));
}

static void test_cfg_secret_writable() {
  Settings base = defaults();
  Settings out;
  Command c;
  ASSERT_OK(parse("{\"t\":\"cfg\",\"apPass\":\"nowehaslo99\"}", c, &base, &out));
  TEST_ASSERT_EQUAL_STRING("nowehaslo99", out.apPass);
  TEST_ASSERT_EQUAL_UINT64(bitOf("apPass"), c.cfgMask);
  ASSERT_ERR(parse("{\"t\":\"cfg\",\"apPass\":\"short\"}", c, &base, &out), Err::OutOfRange, "apPass");
}

static void test_cfg_every_field_once() {
  // A patch that touches every field (two frames to stay under 256 bytes).
  Settings base = defaults();
  Settings out;
  Command c;
  const char* a =
      "{\"t\":\"cfg\",\"rotorMax\":90,\"rotorMin\":5,\"rotorRamp\":800,\"rotorDuty\":200,"
      "\"rotorRev\":true,\"rotorMaxRun\":60,\"yawSpeed\":50,\"yawDuty\":180,\"yawInvert\":true,"
      "\"yawMaxBal\":20000,\"yawMs360\":12000,\"yawMaxDeg\":540,\"yawSensRev\":true}";
  ASSERT_OK(parse(a, c, &base, &out));
  TEST_ASSERT_EQUAL_UINT8(90, out.rotorMax);
  TEST_ASSERT_EQUAL_UINT8(5, out.rotorMin);
  TEST_ASSERT_EQUAL_UINT16(800, out.rotorRamp);
  TEST_ASSERT_EQUAL_UINT8(200, out.rotorDuty);
  TEST_ASSERT_TRUE(out.rotorRev);
  TEST_ASSERT_EQUAL_UINT16(60, out.rotorMaxRun);
  TEST_ASSERT_EQUAL_UINT8(50, out.yawSpeed);
  TEST_ASSERT_EQUAL_UINT8(180, out.yawDuty);
  TEST_ASSERT_TRUE(out.yawInvert);
  TEST_ASSERT_EQUAL_UINT32(20000, out.yawMaxBal);
  TEST_ASSERT_EQUAL_UINT32(12000, out.yawMs360);
  TEST_ASSERT_EQUAL_UINT16(540, out.yawMaxDeg);
  TEST_ASSERT_TRUE(out.yawSensRev);
  const char* b =
      "{\"t\":\"cfg\",\"batType\":2,\"batCal\":0.95,\"batLow\":7200,\"batCrit\":6400,"
      "\"batEmpty\":6300,\"batFull\":9000,\"fsTimeout\":700,\"name\":\"T1\",\"apPass\":\"abcdefgh\","
      "\"apChan\":6,\"txPower\":44,\"bleMode\":2,\"captive\":true,\"led\":false}";
  ASSERT_OK(parse(b, c, &out, &out));
  TEST_ASSERT_EQUAL_UINT8(2, out.batType);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.95f, out.batCal);
  TEST_ASSERT_EQUAL_UINT16(7200, out.batLow);
  TEST_ASSERT_EQUAL_UINT16(6400, out.batCrit);
  TEST_ASSERT_EQUAL_UINT16(6300, out.batEmpty);
  TEST_ASSERT_EQUAL_UINT16(9000, out.batFull);
  TEST_ASSERT_EQUAL_UINT16(700, out.fsTimeout);
  TEST_ASSERT_EQUAL_STRING("T1", out.name);
  TEST_ASSERT_EQUAL_STRING("abcdefgh", out.apPass);
  TEST_ASSERT_EQUAL_UINT8(6, out.apChan);
  TEST_ASSERT_EQUAL_UINT8(44, out.txPower);
  TEST_ASSERT_EQUAL_UINT8(2, out.bleMode);
  TEST_ASSERT_TRUE(out.captive);
  TEST_ASSERT_FALSE(out.led);
  TEST_ASSERT_EQUAL_INT(14, __builtin_popcountll(c.cfgMask));
  TEST_ASSERT_EQUAL_INT(-1, settingsValidate(out));
}

static void test_many_members_still_parse() {
  // Worst case for the parser memory: as many members as fit in 256 bytes.
  // Unique 4-letter keys (longer than ArduinoJson's inline tiny strings).
  std::string s = "{\"t\":\"cfg\"";
  int n = 0;
  while (s.size() + 10 <= proto::kMaxCmdLen - 1) {
    s += ",\"k";
    s += (char)('a' + n / 26);
    s += (char)('a' + n % 26);
    s += "x\":1";
    n++;
  }
  s += "}";
  TEST_ASSERT_TRUE(s.size() <= proto::kMaxCmdLen);
  Command c;
  ASSERT_OK(parseCommand(s.c_str(), s.size(), c));
  TEST_ASSERT_TRUE(c.cfgGet);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_peek_basic);
  RUN_TEST(test_peek_whitespace_and_position);
  RUN_TEST(test_peek_only_top_level_t);
  RUN_TEST(test_peek_exact_value_only);
  RUN_TEST(test_peek_duplicate_last_wins_like_parser);
  RUN_TEST(test_peek_garbage_and_bounds);
  RUN_TEST(test_peek_agrees_with_parser);
  RUN_TEST(test_too_long);
  RUN_TEST(test_bad_json);
  RUN_TEST(test_bad_t);
  RUN_TEST(test_unknown_cmd);
  RUN_TEST(test_simple_commands);
  RUN_TEST(test_id);
  RUN_TEST(test_id_kept_on_field_error);
  RUN_TEST(test_seq);
  RUN_TEST(test_command_is_reset);
  RUN_TEST(test_rotor_speed);
  RUN_TEST(test_rotor_errors);
  RUN_TEST(test_rotor_brake);
  RUN_TEST(test_yaw_dir);
  RUN_TEST(test_yaw_dir_errors);
  RUN_TEST(test_yaw_hold);
  RUN_TEST(test_yaw_goto);
  RUN_TEST(test_yaw_zero_and_cal);
  RUN_TEST(test_yaw_exactly_one_action);
  RUN_TEST(test_cfg_get_and_reset);
  RUN_TEST(test_cfg_set_applies_patch);
  RUN_TEST(test_cfg_patch_is_atomic);
  RUN_TEST(test_cfg_in_place_patch);
  RUN_TEST(test_cfg_battery_preset);
  RUN_TEST(test_cfg_cross_field_validation);
  RUN_TEST(test_cfg_without_base_only_checks);
  RUN_TEST(test_cfg_secret_writable);
  RUN_TEST(test_cfg_every_field_once);
  RUN_TEST(test_many_members_still_parse);
  return UNITY_END();
}
