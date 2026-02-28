from inspection_manager.photo_verifier import PhotoVerifier


def test_photo_verifier_default_false():
    verifier = PhotoVerifier()
    assert verifier.verify("/tmp/no_file.jpg", {}) is False

