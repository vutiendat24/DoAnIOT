

from fastapi import Header, HTTPException
from typing import Optional
import logging

logger = logging.getLogger(__name__)


async def verify_token(authorization: Optional[str] = Header(None)) -> str:
   
    if not authorization:
        raise HTTPException(
            status_code=401,
            detail="Authorization header required"
        )
    
    try:
        
        scheme, token = authorization.split()
        
        if scheme.lower() != 'bearer':
            raise HTTPException(
                status_code=401,
                detail="Invalid authentication scheme"
            )
        
        if token == "test_token":
            return "test_user_123"
        
        
        raise HTTPException(
            status_code=401,
            detail="Invalid token"
        )
        
    except ValueError:
        raise HTTPException(
            status_code=401,
            detail="Invalid authorization header format"
        )
    except Exception as e:
        logger.error(f"Token verification error: {str(e)}")
        raise HTTPException(
            status_code=401,
            detail="Token verification failed"
        )