from amr_perception.contracts import ObservationHeader, ObjectProposal, GraspProposal, to_jsonable


def test_object_proposal_is_only_usable_when_fresh_and_confident():
    proposal = ObjectProposal(
        label="cup",
        confidence=0.82,
        header=ObservationHeader(frame_id="camera_color_optical_frame", age_sec=0.2),
    )

    assert proposal.usable is True
    assert to_jsonable(proposal)["proposal_only"] is True


def test_grasp_proposal_is_never_executable_by_itself():
    proposal = GraspProposal(
        object_label="cup",
        confidence=0.9,
        header=ObservationHeader(frame_id="base_link", age_sec=0.1),
        grasp_frame="base_link",
        approach_vector_xyz=(0.0, 0.0, -1.0),
    )

    assert proposal.executable is False
