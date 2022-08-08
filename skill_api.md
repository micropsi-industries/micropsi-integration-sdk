
# API to trigger execution of a skill on mirai from an external system (e.g. a robot program or PLC)

## Skill Execution Workflow:
### Setup:
* Configure connection (IP/Port)
* Get a list of available skills
### Execute:
* Optionally preload a skill
* Trigger the execution of a skill
### Assess:
* Optionally check for result
* Optionally check for last endstate values

## API methods:
* `get_box_metadata()`
    Return metadata for this box, currently: boxID, configured backend, skill count
* `get_trained_skills()`
    Return skill_id and name of available skills
* `prepare_skill_async(skill_id)`
    Preload the skill with the given skill_id for faster initialization of the `execute_skill` call
* `execute_skill(skill_id)`
    Trigger execution of the skill with the given skill_id
* `get_result(skill_id)`
    Return success or failure depending on the result of the skill execution
* `get_last_endstate_values(skill_id)`
    Returns the last recorded values of the endstate-related parameters as a list: speed, force, done_probability

These methods are exposed via a [JSON API](#JSON), an [XMLRPC API](#XMLRPC). In case XML or JSON is not or badly supported, there is also a [binary API](#Binary-API-V1).

### JSON:

* call: `GET http://$HOST:6543/skills/get_box_metadata`
* successful response: `{"status": "success", "data": [["box_id", 1], ["crunch_url", "https://crunch.micropsi-industries.com/api/"], ["skill_count", 4]]}`
---

* call: `GET http://$HOST:6543/skills/get_trained_skills`
* successful response: `{"status": "success", "data": [[1, "A Skill"], [2, "Another skill"]]}`
---

* call: `POST http://$HOST:6543/skills/execute_skill`
* request payload: `{skill_id: 1}`
* successful response: `{"status": "success", "data": null}`
---

* call: `GET http://$HOST:6543/skills/get_result?skill_id=1`
* successful response: `{"status": "success", "data": null}`
---

* call: `GET http://$HOST:6543/skills/get_last_endstate_values?skill_id=1`
* successful response: `{"status": "success", "data": [0.0, 0.0, 1.0]}`
---

unsuccessful response for all calls: `{"status": "error", "data": "some error message"}`

### XMLRPC:

xmlrpc calls are all calls against `POST http://$HOST:6543/skills/xmlrpc` and follow the official spec: http://xmlrpc.scripting.com/spec.html

request payload: `<?xml version="1.0"?><methodCall><methodName>get_box_metadata</methodName></methodCall>`
successful response:
```
<?xml version='1.0'?>
<methodResponse>
<params>
<param>
<value><array><data>
<value><array><data>
<value><string>box_id</string></value>
<value><int>1</int></value>
</data></array></value>
<value><array><data>
<value><string>crunch_url</string></value>
<value><string>http://crunch-infra:8080/api/</string></value>
</data></array></value>
<value><array><data>
<value><string>skill_count</string></value>
<value><int>1</int></value>
</data></array></value>
</data></array></value>
</param>
</params>
</methodResponse>
```
---

request payload: `<?xml version="1.0"?><methodCall><methodName>get_trained_skills</methodName></methodCall>`
successful response:
```
<?xml version='1.0'?>
<methodResponse>
<params>
<param>
<value><array><data>
<value><array><data>
<value><int>2</int></value>
<value><string>Another skill</string></value>
</data></array></value>
<value><array><data>
<value><int>1</int></value>
<value><string>A skill</string></value>
</data></array></value>
</data></array></value>
</param>
</params>
</methodResponse>
```
---

request payload: `<?xml version="1.0"?><methodCall><methodName>execute_skill</methodName><params><param><value><i4>1</i4></value></param></params></methodCall>`
successful response:
```
<?xml version='1.0'?>
<methodResponse>
<params>
<param>
<value><string>Success</string></value>
</param>
</params>
</methodResponse>
```
---

request payload: `<?xml version="1.0"?><methodCall><methodName>get_result</methodName><params><param><value><i4>1</i4></value></param></params></methodCall>`
successful response:
```
<?xml version='1.0'?>
<methodResponse>
<params>
<param>
<value><string>Success</string></value>
</param>
</params>
</methodResponse>
```
---

request payload: `<?xml version="1.0"?><methodCall><methodName>get_last_endstate_values</methodName><params><param><value><i4>1</i4></value></param></params></methodCall>`
successful response:
```
<?xml version='1.0'?>
<methodResponse>
<params>
<param>
<value><array><data>
<value><double>0.0</double></value>
<value><double>0.0</double></value>
<value><double>1.0</double></value>
</data></array></value>
</param>
</params>
</methodResponse>
```
---

unsuccessful response for all calls:
```
<?xml version='1.0'?>
<methodResponse>
<fault>
<value><struct>
<member>
<name>faultString</name>
<value><string>Error message</string></value>
</member>
<member>
<name>faultCode</name>
<value><int>500</int></value>
</member>
</struct></value>
</fault>
</methodResponse>
```

### Binary API V1

In case the other side doesn't have convenient capabilities to handle JSON or XML, there's also a
custom binary interface to manage skill execution from remote.
A TCP socket is listening on the mirai controller on port `6599`, and communicates via utf-8 encoded
[bytestrings](https://docs.python.org/3/library/struct.html) in network byte order.

Request and response messages are always structured as follows:
* char[4]: `MRSI`
* uint32: API Version `1`
* uint32: Message type
* uint32: Total message size _from start of header to end of content_
* message content (varies)

The available message types in version 1:
```
1: 'get_box_metadata'
2: 'get_trained_skills'
3: 'execute_skill'
4: 'prepare_skill_async'
5: 'get_result'
6: 'get_last_endstate_values'
7: 'get_exception_message'
8: 'failure message'
```

Example request: `b'MRSI\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x18'` (`'!4sIII'`: marker, API version, message type, message size).
Example response: `b'MRSI\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x17\x00\x00\x00*\x00\x00\x00\x18'`: (`!4sIIIII`: marker, version, message type, message_size, box id, number of skills)

message type `8` is a failure message. `mirai` will respond with this message type if the request was unsuccesful or caused an error.

---

`get_box_metadata` request

```python
import struct
request = struct.pack(
    '!4sIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    1,  # message type
    16  # message size
)
```

`get_box_metadata` response

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x01\x00\x00\x00\x18\x00\x00\x00{\x00\x00\x00\x02'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
box_id, number_of_skills = struct.unpack('!II', content)
# 123, 2
```

---

`get_trained_skills` request:

```python
import struct
request = struct.pack(
    '!4sIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    2,  # message type
    16  # message size
)
```

`get_trained_skills` response:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x02\x00\x00\x00A\x00\x00\x00\x02\x00\x00\x00\x17\x00\x00\x00\x0cmotion skill\x00\x00\x00*\x00\x00\x00\x11positioning skill'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]

number_of_skills, = struct.unpack('!I', content[0:4])  # 4 bytes announcing the number of trained skills
offset = 4
skills = []
for i in range(number_of_skills):
    skill_id, = struct.unpack('!I', content[offset:offset+4])  # skill ID as INT
    offset += 4
    name_length, = struct.unpack('!I', content[offset:offset+4])  # length of skill name as INT
    offset += 4
    skill_name, = struct.unpack('!%ds' % name_length, content[offset:offset + name_length])
    offset += name_length
    skills.append({
        'id': skill_id,
        'name': skill_name.decode()
    })
```

---

`execute_skill` request:

```python
import struct
request = struct.pack(
    '!4sIIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    3,  # message type
    20,  # message size
    42  # skill ID
)
```

`execute_skill` response:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x03\x00\x00\x00\x11\x01'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
result_boolean, = struct.unpack('!?', content)
```


---

`prepare_skill_async` request:

```python
import struct
request = struct.pack(
    '!4sIIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    4,  # message type
    20,  # message size
    42  # skill ID
)
```

`execute_skill` response:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x04\x00\x00\x00\x11\x01'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
result_boolean, = struct.unpack('!?', content)
```

---

`get_result` request:

```python
import struct
request = struct.pack(
    '!4sIIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    5,  # message type
    20,  # message size
    42  # skill ID
)
```

`get_result` responds with a signed integer code for the result of the skill exection:
* `0`: No result to report. Skill not started or still running
* `1`: Skill execution ended by speed-based endstate
* `2`: Skill execution ended by force-based endstate
* `3`: Skill execution ended by visual endstate
* `4`: Skill execution ended by timeout
* `5`: Skill execution ended by position-based endstate
* `-1`: an exception occured during skill execution. The client can retrieve the exception message with a `get_exception_message` request:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x05\x00\x00\x00\x14\x00\x00\x00\x02'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
result_code, = struct.unpack('!i', content)
```

---

`last_endstate_values` request:

```python
import struct
request = struct.pack(
    '!4sIIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    6,  # message type
    20,  # message size
    42  # skill ID
)
```

`last_endstate_values` response:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x06\x00\x00\x00\x1c>L\xcc\xcd>\x99\x99\x9a>\xcc\xcc\xcd'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
speed, force, done_probability = struct.unpack('!fff', content)
```

---

`get_exception_message` request:

```python
import struct
request = struct.pack(
    '!4sIIII',  # format string
    'MRSI'.encode(),  # mark
    1,  # API Version
    7,  # message type
    20,  # message size
    42  # skill ID
)
```

`get_exception_message` response:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x07\x00\x00\x00%\x00\x00\x00\x11exception message'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
message_length, = struct.unpack('!I', content[0:4])
message, = struct.unpack('!%ss' % message_length, content[4:4 + message_length])
message = message.decode()
```

---

example error response:

```python
import struct
response = b'MRSI\x00\x00\x00\x01\x00\x00\x00\x08\x00\x00\x00!\x00\x00\x00\rerror message'
mark, version, msg_type, msg_size = struct.unpack('!4sIII', response[0:16])  # first 16 bytes
content = response[16:]  # remaining bytes
message_length, = struct.unpack('!I', content[0:4])
message, = struct.unpack('!%ss' % message_length, content[4:4 + message_length])
message = message.decode()
```

### Binary API V2

The v2 binary API follows the same structure and exposes the same routes as v1. The API version is
handled dynamically in the mirai runtime for each received message, however interleaving messages
constructed for different API versions is not recommended.

The changes for API v2 are as follows.
- The API Version header section is set to 2 in all requests and responses.
- All requests and responses are terminated with the ascii-encoded terminating bytes `\r\n`. Failure
to terminate API v2 requests with these bytes may cause the mirai controller to silently drop the 
connection.
- All strings (skill names, etc) are ascii-encoded. Non-ascii bytes are dropped when constructing 
responses.

- Request and response messages are always structured as follows:
* char[4]: `MRSI`
* uint32: API Version `2`
* uint32: Message type
* uint32: Total message size _including header, content, and terminating bytes_
* message content
* terminating bytes `\r\n`
