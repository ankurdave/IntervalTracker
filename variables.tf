variable "region" {
  default = "us-east-1"
  description = "AWS region to use for launching services"
}

variable "access_key" {
  description = "AWS access key"
}

variable "secret_key" {
  description = "AWS secret key"
}

variable "coreid" {
  description = "Particle Cloud device id (from https://console.particle.io/devices)"
}

variable "google_api_key" {
  description = "Google Maps API key (from https://developers.google.com/maps/documentation/javascript/get-api-key)"
}

variable "tracker_name" {
  description = "Display name for your Particle device"
}
