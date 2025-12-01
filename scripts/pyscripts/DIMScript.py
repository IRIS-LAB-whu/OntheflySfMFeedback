import torch
from deep_image_matching import Quality, extractors, matchers
from deep_image_matching.extractors.extractor_base import extractor_loader
from deep_image_matching.matchers.matcher_base import matcher_loader

confs = {
    "superpoint+lightglue": {
        "extractor": {
            "name": "superpoint",
            "nms_radius": 3,
            "keypoint_threshold": 0.0005,
            "max_keypoints": 8000,
        },
        "matcher": {
            # Refer to https://github.com/cvg/LightGlue/tree/main for the meaning of the parameters
            "name": "lightglue",
            "n_layers": 9,
            "mp": False,  # enable mixed precision
            "flash": True,  # enable FlashAttention if available.
            "depth_confidence": 0.95,  # early stopping, disable with -1
            "width_confidence": 0.99,  # point pruning, disable with -1
            "filter_threshold": 0.1,  # match threshold
        },
    },
    "superpoint+lightglue_fast": {
        "extractor": {
            "name": "superpoint",
            "nms_radius": 3,
            "keypoint_threshold": 0.001,
            "max_keypoints": 1024,
        },
        "matcher": {
            "name": "lightglue",
            "n_layers": 7,
            "mp": False,  # enable mixed precision
            "flash": True,  # enable FlashAttention if available.
            "depth_confidence": 0.95,  # early stopping, disable with -1
            "width_confidence": 0.99,  # point pruning, disable with -1
            "filter_threshold": 0.1,  # match threshold
        },
    },
    "superpoint+superglue": {
        "extractor": {
            "name": "superpoint",
            "nms_radius": 3,
            "keypoint_threshold": 0.0005,
            "max_keypoints": 4096,
        },
        "matcher": {
            "name": "superglue",
            "weights": "outdoor",
            "match_threshold": 0.3,
            "sinkhorn_iterations": 100,
        },
    },
    "disk+lightglue": {
        "extractor": {
            "name": "disk",
            "max_keypoints": 4096,
        },
        "matcher": {
            "name": "lightglue",
        },
    },
    "aliked+lightglue": {
        "extractor": {
            "name": "aliked",
            "model_name": "aliked-n16rot",
            "max_num_keypoints": 4000,
            "detection_threshold": 0.2,
            "nms_radius": 3,
        },
        "matcher": {
            "name": "lightglue",
            "n_layers": 9,
            "depth_confidence": 0.95,  # early stopping, disable with -1
            "width_confidence": 0.99,  # point pruning, disable with -1
            "filter_threshold": 0.1,  # match threshold
        },
    },
    "orb+kornia_matcher": {
        "extractor": {
            "name": "orb",
        },
        "matcher": {"name": "kornia_matcher", "match_mode": "snn"},
    },
    "sift+kornia_matcher": {
        "extractor": {
            "name": "sift",
        },
        "matcher": {"name": "kornia_matcher", "match_mode": "smnn", "th": 0.85},
    },
    "loftr": {
        "extractor": {"name": "no_extractor"},
        "matcher": {"name": "loftr", "pretrained": "outdoor"},
    },
    "se2loftr": {
        "extractor": {"name": "no_extractor"},
        "matcher": {"name": "se2loftr", "pretrained": "outdoor"},
    },
    "roma": {
        "extractor": {"name": "no_extractor"},
        "matcher": {"name": "roma", "pretrained": "outdoor"},
    },
    "keynetaffnethardnet+kornia_matcher": {
        "extractor": {
            "name": "keynetaffnethardnet",
            "n_features": 4000,
            "upright": False,
        },
        "matcher": {"name": "kornia_matcher", "match_mode": "smnn", "th": 0.95},
    },
    "dedode+kornia_matcher": {
        "extractor": {
            "name": "dedode",
            "n_features": 1000,
            "upright": False,
        },
        "matcher": {"name": "kornia_matcher", "match_mode": "smnn", "th": 0.99},
    },
}

# import sys


# class StdStream:
#     def write(self, arg): pass
    
#     def flush(self): pass


# sys.stdout = StdStream()
# sys.stderr = StdStream()


class OTFMatcher:
    def __init__(self, pipeline='superpoint+lightglue'):
        self.config = confs[pipeline]  # type: dict[str,dict]
        self.local_features = self.config['extractor']['name']
        self.matching_method = self.config['matcher']['name']
        self.config["general"] = {"Quality": Quality.MEDIUM}
        self.features_dict_store = {}
        
        try:
            Extractor = extractor_loader(extractors, self.local_features)
        except AttributeError:
            raise ValueError(
                f"Invalid local feature extractor. {self.local_features} is not supported."
            )
        self.extractor = Extractor(self.config)
        
        try:
            Matcher = matcher_loader(matchers, self.matching_method)
        except AttributeError:
            raise ValueError(
                f"Invalid matcher. {self.matching_method} is not supported."
            )
        if self.matching_method == "lightglue":
            self.matcher = Matcher(
                local_features=self.local_features, config=self.config
            )
        else:
            self.matcher = Matcher(self.config)
    
    def extract(self, path):
        path = path.replace('\\', '/')
        features_dict = self.extractor.extract_row(path)  # type:dict
        torch.cuda.empty_cache()
        return features_dict
    
    def match(self, features1, features2):
        return self.matcher.match_row(features1, features2)
    
    def save_features(self, name, feature_dict):
        self.features_dict_store[name] = feature_dict
    
    def read_features(self, name):
        return self.features_dict_store[name]


def main():
    otf_matcher = OTFMatcher()
    feat0 = otf_matcher.extract(r"D:\CS\study\Lightglue\pyscript\images\00001.jpg")
    feat1 = otf_matcher.extract(r"D:\CS\study\Lightglue\pyscript\images\00002.jpg")
    matches = otf_matcher.match(feat0, feat1)
    print(type(matches[0][0]))
    print(len(feat0['keypoints']), len(feat1['keypoints']), len(matches))


def otf_extract(img_path: str):
    otf_matcher = OTFMatcher('superpoint+lightglue')
    return otf_matcher.extract(img_path)


if __name__ == '__main__':
    main()
