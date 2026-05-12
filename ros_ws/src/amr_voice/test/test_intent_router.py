from amr_voice.intent_router import GO_TO_PLACE, READ_STATUS, UNKNOWN, parse_router_json


def test_parse_router_json_accepts_strict_json():
    route = parse_router_json(
        '{"intent":"read_status","confidence":0.92,"arguments":{},"requires_confirmation":false}'
    )

    assert route.intent == READ_STATUS
    assert route.confidence == 0.92
    assert route.requires_confirmation is False


def test_parse_router_json_extracts_json_from_model_text():
    route = parse_router_json(
        'Here is the JSON: {"intent":"go_to_place","confidence":0.82,'
        '"arguments":{"place":"kitchen"},"requires_confirmation":false}'
    )

    assert route.intent == GO_TO_PLACE
    assert route.arguments["place"] == "kitchen"
    assert route.requires_confirmation is True


def test_parse_router_json_rejects_unknown_intent_name():
    route = parse_router_json(
        '{"intent":"launch_missiles","confidence":1.0,"arguments":{},"requires_confirmation":false}'
    )

    assert route.intent == UNKNOWN
    assert route.confidence == 1.0


def test_parse_router_json_treats_valid_zero_confidence_as_usable():
    route = parse_router_json(
        '{"intent":"read_status","confidence":0.0,"arguments":{},"requires_confirmation":false}'
    )

    assert route.intent == READ_STATUS
    assert route.confidence == 0.6
