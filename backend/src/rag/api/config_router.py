"""Configuration router for agent API endpoints."""

from fastapi import APIRouter, HTTPException
from typing import Dict
from ..services.config_service import ConfigService
from ..models.configuration import AgentConfiguration


router = APIRouter(prefix="/config", tags=["config"])


# Global config service instance
config_service = ConfigService()


@router.put("/", response_model=Dict)
async def update_global_config(config: Dict):
    """Update global configuration parameters."""
    try:
        is_valid = config_service.validate_config_dict(config)
        if not is_valid:
            raise HTTPException(status_code=400, detail="Invalid configuration parameters")
        
        success = config_service.update_global_defaults(config)
        if success:
            return {"status": "success", "message": "Global configuration updated"}
        else:
            raise HTTPException(status_code=500, detail="Failed to update global configuration")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error updating global config: {str(e)}")


@router.get("/", response_model=Dict)
async def get_global_config():
    """Retrieve the current global configuration."""
    try:
        default_config = config_service.get_default_config()
        return {"config": default_config.dict()}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving global config: {str(e)}")


@router.post("/validate", response_model=Dict)
async def validate_config(config: Dict):
    """Validate configuration parameters."""
    try:
        is_valid = config_service.validate_config_dict(config)
        return {
            "is_valid": is_valid,
            "message": "Configuration is valid" if is_valid else "Configuration has validation errors"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error validating config: {str(e)}")


@router.post("/normalize", response_model=Dict)
async def normalize_config(config: Dict):
    """Normalize configuration parameters to be within valid ranges."""
    try:
        # Create a temporary AgentConfiguration to normalize
        temp_config = AgentConfiguration(**config)
        
        # The AgentConfiguration model already validates and constrains the values
        # So just return the normalized config
        normalized_config = config_service.normalize_config_range(temp_config)
        
        return {
            "normalized_config": normalized_config.dict(),
            "message": "Configuration normalized to valid ranges"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error normalizing config: {str(e)}")


@router.post("/merge", response_model=Dict)
async def merge_config(payload: Dict):
    """Merge base configuration with override parameters."""
    try:
        base_config_data = payload.get("base_config", {})
        override_config = payload.get("override_config", {})
        
        if not base_config_data:
            base_config = config_service.get_default_config()
        else:
            base_config = AgentConfiguration(**base_config_data)
        
        merged_config = config_service.merge_configs(base_config, override_config)
        
        return {
            "merged_config": merged_config.dict(),
            "message": "Configurations merged successfully"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error merging configs: {str(e)}")