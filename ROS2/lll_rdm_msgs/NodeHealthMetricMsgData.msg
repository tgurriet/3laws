string OK="OK"
string SOME_TOPICS_TIMEOUT="SOME_TOPICS_TIMEOUT"
string TIMING_ISSUE="TIMING_ISSUE"
string ALL_TOPICS_TIMEOUT="ALL_TOPICS_TIMEOUT"



string node_id
int64 time_since_last_msg
bool ok
string error_code
ProcessHealthMetricMsgData[<=1] process
NodeHealthMetricMsgTopicData[] topics
