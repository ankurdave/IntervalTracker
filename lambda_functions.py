import boto3
def insert_into_dynamo(event, context):
    print event
    item = {k: {'S': v} for k, v in event.iteritems()}
    client = boto3.client('dynamodb')
    try:
        client.put_item(TableName='tracker', Item=item)
    except Exception, e:
        print (e)
    return event
