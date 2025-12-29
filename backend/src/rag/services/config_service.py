"""Configuration service for managing agent behavior parameters."""

from typing import Dict
from ..models.configuration import AgentConfiguration


class ConfigService:
    """Service to manage agent configuration parameters."""
    
    def __init__(self, default_config: AgentConfiguration = None):
        """Initialize the configuration service."""
        self.default_config = default_config or AgentConfiguration()
        self.configurations: Dict[str, AgentConfiguration] = {}
    
    def get_default_config(self) -> AgentConfiguration:
        """Get the default agent configuration."""
        return self.default_config
    
    def update_global_defaults(self, new_defaults: Dict) -> bool:
        """Update global default configuration values."""
        try:
            for key, value in new_defaults.items():
                if hasattr(self.default_config, key):
                    setattr(self.default_config, key, value)
            return True
        except Exception:
            return False
    
    def validate_config(self, config: AgentConfiguration) -> bool:
        """Validate a configuration object."""
        try:
            # The configuration is valid if it's an AgentConfiguration object
            # since Pydantic handles validation on construction and field assignment
            return True
        except Exception:
            return False
    
    def validate_config_dict(self, config_dict: Dict) -> bool:
        """Validate a configuration dictionary."""
        try:
            # Try to create an AgentConfiguration object from the dict
            AgentConfiguration(**config_dict)
            return True
        except Exception:
            return False
    
    def merge_configs(self, base_config: AgentConfiguration, override_config: Dict) -> AgentConfiguration:
        """Merge a base configuration with override parameters."""
        # Create a new config based on the base config
        merged_config_dict = base_config.dict()
        
        # Apply overrides
        for key, value in override_config.items():
            if hasattr(base_config, key):
                merged_config_dict[key] = value
        
        # Create and return new AgentConfiguration
        return AgentConfiguration(**merged_config_dict)
    
    def normalize_config_range(self, config: AgentConfiguration) -> AgentConfiguration:
        """Normalize configuration parameters to be within valid ranges."""
        # Apply min/max constraints to configuration parameters
        config.temperature = max(0.0, min(1.0, config.temperature))
        config.grounding_strictness = max(0.0, min(1.0, config.grounding_strictness))
        config.retrieval_top_k = max(1, min(20, config.retrieval_top_k))
        config.similarity_threshold = max(0.0, min(1.0, config.similarity_threshold))
        
        return config
    
    def get_config_diff(self, config1: AgentConfiguration, config2: AgentConfiguration) -> Dict:
        """Get the differences between two configuration objects."""
        diff = {}
        config1_dict = config1.dict()
        config2_dict = config2.dict()
        
        for key in config1_dict:
            if config1_dict[key] != config2_dict[key]:
                diff[key] = {
                    'old': config1_dict[key],
                    'new': config2_dict[key]
                }
        
        return diff