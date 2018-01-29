provider "aws" {
  region     = "${var.region}"
  access_key = "${var.access_key}"
  secret_key = "${var.secret_key}"
  version    = "~> 1.7"
}

provider "archive" {
  version = "~> 1.0"
}

provider "template" {
  version = "~> 1.0"
}

provider "local" {
  version = "~> 1.1"
}

resource "aws_dynamodb_table" "tracker-locations" {
  name           = "tracker"      # Also update lambda_functions.py when renaming this table
  read_capacity  = 5
  write_capacity = 5
  hash_key       = "coreid"
  range_key      = "published_at"

  attribute {
    name = "coreid"
    type = "S"
  }

  attribute {
    name = "published_at"
    type = "S"
  }
}

data "archive_file" "lambda_functions" {
  type        = "zip"
  source_file = "lambda_functions.py"
  output_path = "lambda_functions.zip"
}

data "aws_iam_policy_document" "assume-role-for-apigateway-and-lambda" {
  statement {
    actions = ["sts:AssumeRole"]

    principals {
      type        = "Service"
      identifiers = ["apigateway.amazonaws.com", "lambda.amazonaws.com"]
    }
  }
}

resource "aws_iam_role" "tracker-role" {
  assume_role_policy = "${data.aws_iam_policy_document.assume-role-for-apigateway-and-lambda.json}"
}

resource "aws_iam_role_policy_attachment" "tracker-role-can-execute-lambdas" {
  role       = "${aws_iam_role.tracker-role.name}"
  policy_arn = "arn:aws:iam::aws:policy/AWSLambdaFullAccess"
}

resource "aws_iam_role_policy_attachment" "tracker-role-can-access-dynamodb-from-apigateway" {
  role       = "${aws_iam_role.tracker-role.name}"
  policy_arn = "arn:aws:iam::aws:policy/AmazonDynamoDBReadOnlyAccess"
}

resource "aws_lambda_function" "record-tracker-location" {
  filename         = "${data.archive_file.lambda_functions.output_path}"
  function_name    = "record-tracker-location"
  role             = "${aws_iam_role.tracker-role.arn}"
  handler          = "lambda_functions.insert_into_dynamo"
  runtime          = "python2.7"
  source_code_hash = "${base64sha256(file("${data.archive_file.lambda_functions.output_path}"))}"
}

resource "aws_api_gateway_rest_api" "tracker-api" {
  name = "tracker-api"
}

resource "aws_api_gateway_deployment" "tracker-api-deployment" {
  depends_on = [
    "aws_api_gateway_method.location-history-GET",
    "aws_api_gateway_integration.location-history-GET-from-dynamo",
    "aws_api_gateway_integration_response.location-history-GET",
    "aws_api_gateway_method_response.location-history-GET",
    "aws_api_gateway_method.location-history-POST",
    "aws_api_gateway_integration.location-history-POST-to-lambda",
    "aws_api_gateway_integration_response.location-history-POST",
    "aws_api_gateway_method_response.location-history-POST",
    "aws_api_gateway_method.location-updates-GET",
    "aws_api_gateway_integration.location-updates-GET-from-dynamo",
    "aws_api_gateway_integration_response.location-updates-GET",
    "aws_api_gateway_method_response.location-updates-GET",
  ]

  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  stage_name  = "prod"
}

# location-history
resource "aws_api_gateway_resource" "location-history" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  parent_id   = "${aws_api_gateway_rest_api.tracker-api.root_resource_id}"
  path_part   = "location-history"
}

# location-history: GET
resource "aws_api_gateway_method" "location-history-GET" {
  rest_api_id   = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id   = "${aws_api_gateway_resource.location-history.id}"
  http_method   = "GET"
  authorization = "NONE"
}

resource "aws_api_gateway_integration" "location-history-GET-from-dynamo" {
  rest_api_id             = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id             = "${aws_api_gateway_resource.location-history.id}"
  http_method             = "${aws_api_gateway_method.location-history-GET.http_method}"
  type                    = "AWS"
  integration_http_method = "POST"
  uri                     = "arn:aws:apigateway:${var.region}:dynamodb:action/Query"
  credentials             = "${aws_iam_role.tracker-role.arn}"

  request_templates = {
    "application/json" = <<EOF
{
    "TableName": "${aws_dynamodb_table.tracker-locations.name}",
    "KeyConditionExpression": "coreid = :coreid",
    "ExpressionAttributeValues": {
        ":coreid": {"S": "${var.coreid}"}
    },
    "ScanIndexForward": false,
    "Limit": 1000
}
EOF
  }

  passthrough_behavior = "NEVER"
}

resource "aws_api_gateway_method_response" "location-history-GET" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id = "${aws_api_gateway_resource.location-history.id}"
  http_method = "${aws_api_gateway_method.location-history-GET.http_method}"
  status_code = "200"

  response_models = {
    "application/json" = "Empty"
  }

  response_parameters = {
    "method.response.header.Access-Control-Allow-Origin" = true
  }
}

resource "aws_api_gateway_integration_response" "location-history-GET" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id = "${aws_api_gateway_resource.location-history.id}"
  http_method = "${aws_api_gateway_method.location-history-GET.http_method}"
  status_code = "${aws_api_gateway_method_response.location-history-GET.status_code}"

  response_templates = {
    "application/json" = ""
  }

  response_parameters = {
    "method.response.header.Access-Control-Allow-Origin" = "'*'"
  }

  depends_on = ["aws_api_gateway_integration.location-history-GET-from-dynamo"]
}

# location-history: OPTIONS (for CORS)
module "location-history-CORS" {
  source      = "github.com/carrot/terraform-api-gateway-cors-module"
  resource_id = "${aws_api_gateway_resource.location-history.id}"
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
}

# location-history: POST
resource "aws_api_gateway_method" "location-history-POST" {
  rest_api_id   = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id   = "${aws_api_gateway_resource.location-history.id}"
  http_method   = "POST"
  authorization = "NONE"
}

resource "aws_api_gateway_integration" "location-history-POST-to-lambda" {
  rest_api_id             = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id             = "${aws_api_gateway_resource.location-history.id}"
  http_method             = "${aws_api_gateway_method.location-history-POST.http_method}"
  type                    = "AWS"
  integration_http_method = "POST"
  uri                     = "arn:aws:apigateway:${var.region}:lambda:path/2015-03-31/functions/${aws_lambda_function.record-tracker-location.arn}/invocations"
  credentials             = "${aws_iam_role.tracker-role.arn}"
  passthrough_behavior    = "WHEN_NO_TEMPLATES"
}

resource "aws_api_gateway_integration_response" "location-history-POST" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id = "${aws_api_gateway_resource.location-history.id}"
  http_method = "${aws_api_gateway_method.location-history-POST.http_method}"
  status_code = "${aws_api_gateway_method_response.location-history-POST.status_code}"

  response_templates = {
    "application/json" = ""
  }

  depends_on = ["aws_api_gateway_integration.location-history-POST-to-lambda"]
}

resource "aws_api_gateway_method_response" "location-history-POST" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id = "${aws_api_gateway_resource.location-history.id}"
  http_method = "${aws_api_gateway_method.location-history-POST.http_method}"
  status_code = "200"

  response_models = {
    "application/json" = "Empty"
  }
}

# location-updates
resource "aws_api_gateway_resource" "location-updates" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  parent_id   = "${aws_api_gateway_rest_api.tracker-api.root_resource_id}"
  path_part   = "location-updates"
}

# location-updates: GET
resource "aws_api_gateway_method" "location-updates-GET" {
  rest_api_id   = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id   = "${aws_api_gateway_resource.location-updates.id}"
  http_method   = "GET"
  authorization = "NONE"

  request_parameters = {
    "method.request.querystring.last_published_at" = true
  }

  request_validator_id = "${aws_api_gateway_request_validator.location-updates-GET-validate-querystring.id}"
}

resource "aws_api_gateway_request_validator" "location-updates-GET-validate-querystring" {
  rest_api_id                 = "${aws_api_gateway_rest_api.tracker-api.id}"
  name                        = "location-updates-GET-validate-querystring"
  validate_request_parameters = true
}

resource "aws_api_gateway_integration" "location-updates-GET-from-dynamo" {
  rest_api_id             = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id             = "${aws_api_gateway_resource.location-updates.id}"
  http_method             = "${aws_api_gateway_method.location-updates-GET.http_method}"
  type                    = "AWS"
  integration_http_method = "POST"
  uri                     = "arn:aws:apigateway:${var.region}:dynamodb:action/Query"
  credentials             = "${aws_iam_role.tracker-role.arn}"

  request_templates = {
    "application/json" = <<EOF
{
    "TableName": "${aws_dynamodb_table.tracker-locations.name}",
    "KeyConditionExpression": "coreid = :coreid AND published_at > :last_published_at",
    "ExpressionAttributeValues": {
        ":coreid": {"S": "${var.coreid}"},
        ":last_published_at": {"S": "$input.params('last_published_at')"}
    },
    "ScanIndexForward": false,
    "Limit": 1000
}
EOF
  }

  passthrough_behavior = "NEVER"
}

resource "aws_api_gateway_integration_response" "location-updates-GET" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id = "${aws_api_gateway_resource.location-updates.id}"
  http_method = "${aws_api_gateway_method.location-updates-GET.http_method}"
  status_code = "${aws_api_gateway_method_response.location-updates-GET.status_code}"

  response_templates = {
    "application/json" = ""
  }

  response_parameters = {
    "method.response.header.Access-Control-Allow-Origin" = "'*'"
  }

  depends_on = ["aws_api_gateway_integration.location-updates-GET-from-dynamo"]
}

resource "aws_api_gateway_method_response" "location-updates-GET" {
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
  resource_id = "${aws_api_gateway_resource.location-updates.id}"
  http_method = "${aws_api_gateway_method.location-updates-GET.http_method}"
  status_code = "200"

  response_models = {
    "application/json" = "Empty"
  }

  response_parameters = {
    "method.response.header.Access-Control-Allow-Origin" = true
  }
}

# location-updates: OPTIONS (for CORS)
module "location-updates-CORS" {
  source      = "github.com/carrot/terraform-api-gateway-cors-module"
  resource_id = "${aws_api_gateway_resource.location-updates.id}"
  rest_api_id = "${aws_api_gateway_rest_api.tracker-api.id}"
}

# Output
locals {
  location_history_url = "https://${aws_api_gateway_deployment.tracker-api-deployment.rest_api_id}.execute-api.${var.region}.amazonaws.com/${aws_api_gateway_deployment.tracker-api-deployment.stage_name}${aws_api_gateway_resource.location-history.path}"
}

output "location_history_url" {
  value = "${local.location_history_url}"
}

data "template_file" "view_html_template" {
  template = "${file("view.html.template")}"

  vars {
    location_history_url = "${local.location_history_url}"
    location_updates_url = "https://${aws_api_gateway_deployment.tracker-api-deployment.rest_api_id}.execute-api.${var.region}.amazonaws.com/${aws_api_gateway_deployment.tracker-api-deployment.stage_name}${aws_api_gateway_resource.location-updates.path}"
    google_api_key       = "${var.google_api_key}"
    tracker_name         = "${var.tracker_name}"
  }
}

resource "local_file" "view_html" {
  content  = "${data.template_file.view_html_template.rendered}"
  filename = "view.html"
}
