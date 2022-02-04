#!/usr/bin/env python3

import boto3
from botocore.config import Config


def gather_list_of_images(ecr_client):
    images = []
    response = ecr_client.describe_repositories(registryId="080653068785")
    for repo in response['repositories']:
        images.append(repo['repositoryName'])
    return images


def gather_untagged_images(ecr_client, images):
    results = {}
    for image in images:
        empty_tags = []
        image_versions = ecr_client.describe_images(repositoryName=image)
        for instance in image_versions["imageDetails"]:
            if "imageTags" not in instance.keys():
                empty_tags.append({"imageDigest":instance["imageDigest"]})
        if empty_tags:
            results[image] = empty_tags
    return results


def delete_images(ecr_client, image_data):
    for image, empty_tag_list in image_data.items():
        print(f"From image: {image} removing images:\n{empty_tag_list}")
        ecr_client.batch_delete_image(repositoryName=image, imageIds=empty_tag_list)


if __name__ == "__main__":
    config = Config(region_name="us-east-1")
    ecr_client = boto3.client('ecr-public',config=config)
    images = gather_list_of_images(ecr_client)
    empty_tags = gather_untagged_images(ecr_client, images)
    delete_images(ecr_client, empty_tags)
