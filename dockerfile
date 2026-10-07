FROM python:3.11-slim

# PyBullet se compila al instalarse, por eso hace falta el compilador
RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8000
EXPOSE 4210/udp

CMD ["python", "-u", "twin.py"]
