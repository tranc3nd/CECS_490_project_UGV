# Use the base Python image from Bookworm
FROM arm64v8/python:3.8-bullseye


# Set the working directory inside the container
WORKDIR /app

# Copy the Python requirements file into the container
COPY ./requirements.txt /app

# Install Python dependencies from requirements.txt
RUN pip install -r requirements.txt
#RUN pip install --no-cache-dir RPi.GPIO

# Copy the rest of the application into the container
COPY . /app/

# Expose port 8000 for the application
EXPOSE 8000

# Environment variable to ensure Python output is sent straight to terminal/container logs
ENV PYTHONUNBUFFERED 1
ENV GPIOZERO_PIN_FACTORY=rpigpio

# Command to run the gunicorn server for your web application
CMD ["gunicorn" , "-b", "0.0.0.0:8000", "--chdir", "/app/main_ugv/com/", "main:app" , "--threads=9"]

