"""HTTP access boundary. Use an SSH tunnel; HTTP Basic is not encryption."""
import hmac
from urllib.parse import urlsplit


def authorized(credentials, password):
    if not password:
        return True  # Local-only default; SSH supplies remote authentication.
    return bool(credentials and credentials.type == 'basic'
                and credentials.username == 'operator'
                and hmac.compare_digest((credentials.password or '').encode(), password.encode()))


def same_origin(origin, host_url):
    if origin is None:
        return True  # CLI tools have no Origin; authentication still applies.
    try:
        a, b = urlsplit(origin), urlsplit(host_url)
        return a.scheme in ('http','https') and (a.scheme,a.netloc)==(b.scheme,b.netloc)
    except ValueError:
        return False
